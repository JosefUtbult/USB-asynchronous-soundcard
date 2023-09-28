#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use rtic_stm32f4xx as _; // global logger + panicking-behavior + memory layout
use stm32f4xx_hal as hal;
use usb_asynchronous_soundcard::*;

use hal::prelude::*;
use hal::otg_fs::{UsbBus, USB};

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

pub const USB_BUFFER_SIZE: usize = 1024;
// This part of memory acts as a buffer for the usb device
static mut EP_MEMORY: [u32; USB_BUFFER_SIZE] = [0; USB_BUFFER_SIZE];

type UsbDevice = usb_device::prelude::UsbDevice<'static, UsbBus<USB>>;
type UsbAudio = usbd_audio::AudioClass<'static, UsbBus<USB>>;

#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI0, EXTI1, EXTI2]
)]

mod app {
    use super::*;

    // Shared resources go here
    #[shared]
    struct Shared {
    }

    // Local resources go here
    #[local]
    struct Local {
        usb_device: UsbDevice,
        usb_audio: UsbAudio
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        // Setup syscfg
        // let mut syscfg = cx.device.SYSCFG.constrain();

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

        let gpioa = cx.device.GPIOA.split();
        // let gpiob = cx.device.GPIOB.split();
        // let gpioc = cx.device.GPIOC.split();

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
        let usb_audio = AudioClassBuilder::new()
            // .input(
                // 	StreamConfig::new_discrete(
                // 	// Signed 24 bit little endian
                // 	Format::S24le,
                // 	1,
                // 	&[48000],
                // 	TerminalType::InMicrophone).unwrap())
            .output(
                StreamConfig::new_discrete(
                // Signed 16 bit little endian
                Format::S16le,
                1,
                &[48000],
                TerminalType::OutSpeaker).unwrap())
            .build(usb_bus)
            .unwrap();

        // Now create a new USB device with ID numbers and
        // manufacturer info
        let usb_device: usb_device::prelude::UsbDevice<UsbBus<USB>> = 
            UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Josef Labs")
                .product("USB audio test")
                .serial_number("42")
                .build();


        //Kickstart the sync interrupt
        usb_audio.write_synch_interrupt(&sample_rate_to_buffer(FfCounter::ZERO)).ok();

        (
            Shared {
            },
            Local {
                usb_device,
                usb_audio
            },
        )
    }


    #[task(
        priority = 2,
        binds = OTG_FS,
        local = [
            usb_device,
            usb_audio
        ]
    )]
    fn usb_transmit_task(cx: usb_transmit_task::Context) {
        // Check so that an USB event has occurred in relation to the
        // Audio Class device
        if cx.local.usb_device.poll(&mut [cx.local.usb_audio]) {
            let mut buf = [0u8; USB_BUFFER_SIZE];
            // Check if the endpoint where the device receives sample
            // data has been written to
            if let Ok(len) = cx.local.usb_audio.read(&mut buf) {
                if len != 96 {
                    defmt::info!("{}", len);
                }
            }
            // If not, that means that the host has read the contents
            // of the synchronous feedback endpoint. In that case,
            // repopulate the endpoint with an appropriate rate
            else {
                // Express a constant rate of 47.5 kHz in fixed decimal format
                let constant_rate = FfCounter::lit("47.50");
                let buf = sample_rate_to_buffer(constant_rate);
                cx.local.usb_audio.write_synch_interrupt(&buf).ok();
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
