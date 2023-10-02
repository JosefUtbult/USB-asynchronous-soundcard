#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use rtic_stm32f4xx::{
    self as _, 
    codec
};

use stm32f4xx_hal as hal;
use usb_asynchronous_soundcard::*;

use hal::gpio::{Pin, Output};
use hal::prelude::*;
use hal::otg_fs::{UsbBus, USB};
use hal::pac::EXTI;

use usb_device::device::{
	UsbDeviceBuilder, 
	UsbVidPid
};

use usbd_audio::{
	AudioClassBuilder, 
	StreamConfig, 
	TerminalType, 
	Format
};

/// This is the rate at which a sync request should be requested by the
/// host. This number is correlated to the `p` value, determined by the
/// relationship between the master clock (we use the left-right clock
/// for this), and the sample rate according to the formula
/// 
/// ```Fm = F_s * 2^p```
/// 
/// For our example, we will get an exponent `p = 1`. This results in
/// a sync rate according to the following formula
/// 
/// ```R = Fs * 2^(10 - p) = 512 ms```
const SYNC_RATE: u32 = 512;

/// This is the size allocated to the USB buffer. As this example
/// utilizes isochronous endpoints, these messages can be up to 1024 bytes
const USB_BUFFER_SIZE: usize = 1024;

/// This part of memory acts as a buffer for the usb device
static mut EP_MEMORY: [u32; USB_BUFFER_SIZE] = [0; USB_BUFFER_SIZE];

/// The USB device defined by the 
/// [usb-device](https://docs.rs/usb-device/latest/usb_device/) crate
type UsbDevice = usb_device::prelude::UsbDevice<'static, UsbBus<USB>>;

/// The USB device class instance defined by the
/// [usbd-audio](https://docs.rs/usbd-audio/latest/usbd_audio/) crate
type UsbAudio = usbd_audio::AudioClass<'static, UsbBus<USB>>;

/// A debug pin used for measuring interrupt rate
type DebugInterruptPin = Pin<'C', 8, Output>;

#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI0, EXTI1, EXTI2]
)]

mod app {
    use usb_asynchronous_soundcard::usbd_audio::SyncConfig;

    use super::*;

    // Shared resources go here
    #[shared]
    struct Shared {
        ff_counter: FfCounter,
    }

    // Local resources go here
    #[local]
    struct Local {
        usb_device: UsbDevice,
        usb_audio: UsbAudio,
        i2s_driver: crate::codec::I2sTransmitDriver,
        exti: EXTI,
        timer: hal::timer::CounterHz<hal::pac::TIM2>,
        debug_pin: DebugInterruptPin,
        usb_to_codec_producer: heapless::spsc::Producer<'static,codec::QueueType,{codec::QUEUE_SIZE}> ,
        usb_to_codec_consumer: heapless::spsc::Consumer<'static,codec::QueueType,{codec::QUEUE_SIZE}> ,
    }

    #[init(
        local = [
            queue: heapless::spsc::Queue<codec::QueueType,{codec::QUEUE_SIZE}>  = heapless::spsc::Queue::new(),
        ]
    )]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let (usb_to_codec_producer, usb_to_codec_consumer) = cx.local.queue.split();

        // Setup clocks and freeze them
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8u32.MHz())
            .sysclk(96.MHz())
            .hclk(96.MHz())
            .pclk1(50.MHz())
            .pclk2(100.MHz())
            .i2s_clk(61440.kHz())
            .freeze();

        let mut syscfg = cx.device.SYSCFG.constrain();
        let mut exti = cx.device.EXTI;

        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();
        let gpioc = cx.device.GPIOC.split();

        let i2c1 = cx.device.I2C1;
        let spi2 = cx.device.SPI2;
        let spi3 = cx.device.SPI3;

        // Declare which pins should be used for the USB interface
        let usb_pins = (
            gpioa.pa11,     // USB DM
            gpioa.pa12      // USB DP
        );

        // Setup a new USB peripheral instance with all its needed
        // pins and clocks and so on
        let usb = hal::otg_fs::USB::new(
            (
                cx.device.OTG_FS_GLOBAL, 
                cx.device.OTG_FS_DEVICE, 
                cx.device.OTG_FS_PWRCLK
            ),
            usb_pins,
            &clocks,
        );

        // Create a new USB bus using the USB peripherals. This USB
        // bus is then applied as a static object, so that it
        // doesn't fall out of scope
        let usb_bus = cortex_m::singleton!(
            : usb_device::class_prelude::UsbBusAllocator<UsbBus<USB>> =
                UsbBus::new(usb, unsafe { &mut EP_MEMORY })
        ).unwrap();

        // Create a USB audio object that is later used in tandem with
        // the USB device to configure this device as an Audio Class instance
        let mut usb_audio = AudioClassBuilder::new()
            .output(
                StreamConfig::new_discrete(
                // Signed 16 bit little endian
                Format::S16le,
                1,
                &[48000],
                TerminalType::OutSpeaker).unwrap())
            // Here the value of `p` is reported to the USB host
            .sync(SyncConfig::new(2))
            .build(usb_bus)
            .unwrap();

        // Now create a new USB device with ID numbers and
        // manufacturer info
        let mut usb_device: usb_device::prelude::UsbDevice<UsbBus<USB>> = 
            UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Josef Labs")
                .product("USB audio test")
                .serial_number("42")
                .build();


        // Wait to start interrupts until a frame of samples arrives.
        // This is to make sure the ff rate calculations won't mess up
        loop {
            // Check so that an USB event has occurred in relation to the
            // Audio Class device
            if usb_device.poll(&mut [&mut usb_audio]) {
                let mut buf = [0u8; USB_BUFFER_SIZE];
                // Check if the endpoint where the device receives sample
                // data has been written to
                if let Ok(_len) = usb_audio.read(&mut buf) {
                    break;
                }
                // If not, that means that the host has read the contents
                // of the synchronous feedback endpoint. In that case,
                // repopulate the endpoint with an appropriate rate
                else {
                    //Kickstart the sync interrupt
                    usb_audio.write_sync_rate(&sample_rate_to_buffer(FfCounter::ZERO)).ok();
                }
            }

        }

        // Setup a debug pin that can be read with a logic analyzer.
        // This pin will be triggered every time we get a timer interrupt
        let debug_pin = gpioc.pc8.into();

        // We use a timer to simulate the interrupt from an I2S
        // connected device
        let mut timer = cx.device.TIM2.counter_hz(&clocks);
        timer.start(96_000_u32.Hz()).unwrap();
        timer.listen(hal::timer::Event::Update);

        let codec = codec::Codec {
            syscfg: &mut syscfg,
            clocks: &clocks,
            exti: &mut exti,
            i2c1: i2c1,
            spi2: spi2,
            spi3: spi3,
            rst_pin: &mut gpioa.pa7.into_push_pull_output(),
            i2c_scl: gpiob.pb8,
            i2c_sda: gpiob.pb9,
            i2s_mclk: gpioc.pc7,
            i2s_lrck: gpioa.pa4,
            i2s_clk: gpioc.pc10,
            i2s_data:gpioc.pc12 
        };

        let i2s_driver = codec::init(codec);

        defmt::info!("Starting");

        (
            Shared {
                ff_counter: FfCounter::ZERO,
            },
            Local {
                usb_device,
                usb_audio,
                i2s_driver,
                exti,
                timer,
                debug_pin,
                usb_to_codec_consumer,
                usb_to_codec_producer
            },
        )
    }

    #[task(
        priority = 4,
        binds = SPI3,
        local = [
            i2s_driver,
            exti,
            frame_state: codec::FrameState = codec::FrameState::LeftMsb,
            frame: codec::QueueType = [0; 4],
            usb_to_codec_producer,
            usb_to_codec_consumer,
            debug_pin
        ],
    )]
    fn i2s_transmit_task(cx: i2s_transmit_task::Context) {
        cx.local.debug_pin.toggle();
        
        codec::i2s_transmit(
            cx.local.frame_state, 
            cx.local.frame, 
            cx.local.usb_to_codec_consumer, 
            cx.local.i2s_driver, 
            cx.local.exti
        );
    }

    /// This timer tick function is triggered at a rate of 96 kHz.
    /// This corresponds to the left-right clock of an I2S device
    /// which is running at 48 kHz. 
    /// 
    /// We need to use this higher rate,
    /// rather than just the 48 kHz interrupt, as the USB
    /// specification for Audio Class devices wants a feedback that is
    /// a multiple of the sample rate according to the function 
    /// 
    /// ```Fm = Fs * 2^p```
    /// 
    /// Where the exponent `p` in this case is 1
    #[task(
        priority = 3,
        binds = TIM2,
        local = [
            timer,
            // debug_pin
        ],
        shared = [
            ff_counter
        ]
    )]
    fn timer_tick(mut cx: timer_tick::Context) {
        cx.local.timer.clear_interrupt(hal::timer::Event::Update);
        // Toggle the debug pin for measuring rate of the interrupt.
        // This rate should be 96 kHz
        // cx.local.debug_pin.toggle();

        // Increment the ff counter by one, which is then used to
        // measure the rate that samples are consumed
        cx.shared.ff_counter.lock(|ff_counter| {
            if *ff_counter < FfCounter::MAX - FfCounter::ONE {
                *ff_counter += FfCounter::ONE;
            }
        });
    }

    /// This interrupt will trigger every time a USB request is sent
    /// to the device. A request can mean a bunch of things, but we
    /// are interested in when a request is sent to device that
    /// corresponds with the audio class. 
    /// 
    /// It starts by calculating the current rate in relation to the
    /// SOF message from the host. This is done every 512 ms.
    /// 
    /// After that it checks if audio sample data has been transmitted
    /// to the device. If not, it means that the host has read the
    /// sync interrupt endpoint. In that case, it logs some
    /// information about the state
    #[task(
        priority = 2,
        binds = OTG_FS,
        shared = [
            ff_counter
        ],
        local = [
            usb_device,
            usb_audio,
            ff: FfCounter = FfCounter::ZERO,
            sample_counter: usize = 0,
            frame_counter: u32 = 0,
            frames_since_sync: u32 = 0
        ]
    )]
    fn usb_transmit_task(mut cx: usb_transmit_task::Context) {
        // Check so that an USB event has occurred in relation to the
        // Audio Class device
        if cx.local.usb_device.poll(&mut [cx.local.usb_audio]) {

            *cx.local.frame_counter += 1;
            *cx.local.frames_since_sync += 1;
            if *cx.local.frame_counter >= SYNC_RATE {
                *cx.local.frame_counter = 0;

                cx.shared.ff_counter.lock(|ff_counter| {
                    let factor: FfCounter = FfCounter::ONE * SYNC_RATE;
                    *cx.local.ff = *ff_counter / factor;
                    *ff_counter %= factor;
                });

                let buf = sample_rate_to_buffer(*cx.local.ff);
                cx.local.usb_audio.write_sync_rate(&buf).ok();
            }

            let mut buf = [0u8; USB_BUFFER_SIZE];
            // Check if the endpoint where the device receives sample
            // data has been written to
            if let Ok(len) = cx.local.usb_audio.read(&mut buf) {
                *cx.local.sample_counter += len;
            }
            // If not, that means that the host has read the contents
            // of the synchronous feedback endpoint. In that case,
            // repopulate the endpoint with an appropriate rate
            else {

                let average_packet_size: f64 = *cx.local.sample_counter as f64 / *cx.local.frames_since_sync as f64;
                defmt::info!(
                    "Rate: {}, Average packet size: {}, Frames since sync: {}", 
                    (*cx.local.ff).to_num::<f32>(), 
                    average_packet_size,
                    *cx.local.frames_since_sync
                );
                *cx.local.sample_counter = 0;
                *cx.local.frames_since_sync = 0;
            }
        }
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

}
