use crate::codecs::cs4272;

use cortex_m::asm;
use stm32f4xx_hal as hal;
// use hal::gpio::Edge;
use hal::i2s::stm32_i2s_v12x::driver::*;
use hal::i2s::I2s;
use hal::pac::{EXTI, SPI2, SPI3};
use hal::prelude::*;
use hal::i2c::I2c;
use hal::gpio::{NoPin, Edge};

use heapless::spsc::*;

use defmt_brtt as _;

pub type I2sMasterDriver = I2sDriver<I2s<SPI2>, Master, Receive, Philips>;
pub type I2sTransmitDriver = I2sDriver<I2s<SPI3>, Slave, Transmit, Philips>;

pub const QUEUE_SIZE: usize = 512;
pub type QueueType = [u16; 4];

// Part of the frame we currently transmit or receive
#[derive(Copy, Clone)]
pub enum FrameState {
    LeftMsb,
    LeftLsb,
    RightMsb,
    RightLsb,
}

impl Default for FrameState {
    fn default() -> Self {
        Self::LeftMsb
    }
}
pub struct Codec<'a> {
    pub syscfg: &'a mut hal::syscfg::SysCfg,
    pub clocks: &'a hal::rcc::Clocks,
    pub exti:   &'a mut EXTI,
    pub i2c1:   hal::pac::I2C1,
    pub spi2:   hal::pac::SPI2,
    pub spi3:   hal::pac::SPI3,
    pub rst_pin: &'a mut hal::gpio::Pin<'A', 7, hal::gpio::Output>,
    pub i2c_scl: hal::gpio::Pin<'B', 8>,
    pub i2c_sda: hal::gpio::Pin<'B', 9>,
    pub i2s_lrck_master: hal::gpio::Pin<'B', 12>,
    pub i2s_clk_master: hal::gpio::Pin<'B', 13>,
    pub i2s_mck_master: hal::gpio::Pin<'C', 6>,
    pub i2s_data_master: hal::gpio::Pin<'B', 15>,
    pub i2s_lrck_transmit: hal::gpio::Pin<'A', 4>,
    pub i2s_clk_transmit: hal::gpio::Pin<'C', 10>,
    pub i2s_data_transmit: hal::gpio::Pin<'C', 12>
}

pub fn init(codec: Codec) -> (I2sMasterDriver, I2sTransmitDriver)
{
    // Pin groups
    let i2c_pins = (
        codec.i2c_scl,
        codec.i2c_sda
    );

    let i2s_master_pins = (
        codec.i2s_lrck_master,
        codec.i2s_clk_master,
        codec.i2s_mck_master,
        codec.i2s_data_master
    );

    let i2s_transmit_pins = (
        codec.i2s_lrck_transmit,
        codec.i2s_clk_transmit,
        NoPin::new(),
        codec.i2s_data_transmit
    );


    // Interface objects
    let mut i2c = I2c::new(codec.i2c1, i2c_pins, 100.kHz(), codec.clocks);
    let i2s_master = I2s::new( codec.spi2, i2s_master_pins, codec.clocks);
    let i2s_transmit = I2s::new(codec.spi3, i2s_transmit_pins, codec.clocks);

    // Setup Master device for configurations codec
    let i2s_master_config = I2sDriverConfig::new_master()
        .receive()
        .standard(Philips)
        .data_format(DataFormat::Data24Channel32)
        .master_clock(true)
        .request_frequency(12_000);
        
    // Setup for the slave transmit
    let i2s_transmit_config = i2s_master_config.transmit().to_slave();

    let mut i2s_master_driver = I2sDriver::new(i2s_master, i2s_master_config);
    defmt::info!("Actual sample rate is {}", i2s_master_driver.sample_rate());
    
    i2s_master_driver.enable();

    // Reset the codec
    defmt::info!("Reset codec");
    codec.rst_pin.set_low();
    for _ in 0..1000000 {
        asm::nop();
    }
    
    codec.rst_pin.set_high();
    for _ in 0..100000 {
        asm::nop();
    }

    cs4272::codec_setup(&mut i2c);

    for _ in 0..10000 {
        asm::nop();
    }

    // Use the slave transmit config to setup a transmit driver
    let mut i2s_transmit_driver = I2sDriver::new(i2s_transmit, i2s_transmit_config);

    // Create an interrupt for the master receive device
    // i2s_master_driver.set_rx_interrupt(true);
    // i2s_master_driver.set_error_interrupt(true);

    // Set interrupts for the slave transmit device
    i2s_transmit_driver.set_tx_interrupt(true);
    i2s_transmit_driver.set_error_interrupt(true);

    // Set up an interrupt on WS pin of the transmit device
    let ws_pin = i2s_transmit_driver.ws_pin_mut();
    ws_pin.make_interrupt_source(codec.syscfg);
    ws_pin.trigger_on_edge(codec.exti, Edge::Rising);
    // We will enable i2s_transmit in interrupt
    ws_pin.enable_interrupt(codec.exti);

    // Wait until device has booted
    for _ in 0..10000 {
        asm::nop();
    }

    // Clear overrun flag
    i2s_master_driver.read_data_register();
    i2s_master_driver.status();

    (i2s_master_driver, i2s_transmit_driver)
}

// Cycle through the different frame states (LeftMsb -> LeftLsb -> RightMsb -> RightLsb ->
// LeftMsb). On start of a frame, pull a new buffer from the Usb to codec queue and for each state,
// push the corresponding value in the buffer to the i2s output
pub fn i2s_transmit(
    frame_state: &mut FrameState, 
    frame: &mut QueueType, 
    usb_to_codec_consumer: &mut Consumer<QueueType, QUEUE_SIZE>, 
    i2s_transmit_driver: &mut I2sTransmitDriver, 
    exti: &mut EXTI
) {
    let status = i2s_transmit_driver.status();
    // it's better to write data first to avoid to trigger udr flag
    if status.txe() {
        let data;
        match (*frame_state, status.chside()) {
            (FrameState::LeftMsb, Channel::Left) => {
                *frame = match usb_to_codec_consumer.dequeue() {
                    Some(frame) => {
                        // defmt::info!("Pulled data from USB");
                        frame
                    },
                    None => {
                        // defmt::info!("Unable to pull data from USB");
                        [0; 4]
                    }
                };
                
                data = frame[0];
                *frame_state = FrameState::LeftLsb;
            }
            (FrameState::LeftLsb, Channel::Left) => {
                data = frame[1];
                *frame_state = FrameState::RightMsb;
            }
            (FrameState::RightMsb, Channel::Right) => {
                data = frame[2];
                *frame_state = FrameState::RightLsb;
            }
            (FrameState::RightLsb, Channel::Right) => {
                data = frame[3];
                *frame_state = FrameState::LeftMsb;
            }
            // in case of udr this resynchronize tracked and actual channel
            _ => {
                *frame_state = FrameState::LeftMsb;
                data = 0; //garbage data to avoid additional underrun
            }
        }
        i2s_transmit_driver.write_data_register(data);
    }
    if status.fre() {
        defmt::warn!("i2s_transmit Frame error");
        i2s_transmit_driver.disable();
        i2s_transmit_driver.ws_pin_mut().enable_interrupt(exti);
    }
    if status.udr() {
        // defmt::dbg!("i2s_transmit udr");
        i2s_transmit_driver.status();
        i2s_transmit_driver.write_data_register(0);
    }
}
