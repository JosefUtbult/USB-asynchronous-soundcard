use stm32f4xx_hal as hal;
use hal::i2c::I2c;

macro_rules! i2c_error_to_string {
    ($err:expr) => {
        match $err {
            hal::i2c::Error::Overrun => {defmt::error!("Error: Overrun")},
            hal::i2c::Error::NoAcknowledge(_) => {defmt::error!("Error: No Acknowledge")},
            hal::i2c::Error::Timeout => {defmt::error!("Error: Timeout")},
            hal::i2c::Error::Bus => {defmt::error!("Error: Bus")},
            hal::i2c::Error::Crc => {defmt::error!("Error: Crc")},
            hal::i2c::Error::ArbitrationLoss => {defmt::error!("Error: Arbitration Loss")}
            _ => todo!()
        }
    };
}

pub fn codec_setup(i2c: &mut I2c<hal::pac::I2C1>) {
    // Write a bunch of commands
    const CODEC_ADDR: u8 = 0b001000_1; // on wire 001000_[ADDR=1]

    let control_1_reg = &[
        0x01u8, // Control Mode 
        // 01 Mode Control 1
        // M1           0   Single-Speed Mode: 4 to 50 kHz sample rates (default)    
        // M0           0
        // Ratio1       0   Single Speed Mode
        // Ratio0       0
        // M/S          1   Master mode
        // DAC_DIF2     0   Right Justified, 16-bit Data DAC input
        // DAC_DIF1     1
        // DAC_DIF0     0
        0b00_00_1_010,
    ];

    let mut buf = *control_1_reg;
    defmt::debug!("control_1_reg {:?}", buf);
    match i2c.write(CODEC_ADDR, &buf) {
        Ok(_) => {
            defmt::debug!("Ok");
        }
        Err(err) => {
            i2c_error_to_string!(err);
            panic!("error {:?}", err);
        }
    }

    let control_2_reg = &[
        0x07u8, // Control Mode 2
        // 07 Mode Control 2
        // Reserved       0
        // Reserved       0
        // Reserved       0
        // LOOP           0    No Loopback
        // MUTE A AND B   0    Individual Mute Signals
        // FREEZE         0    Immediate updates
        // CPEN           1    Enable port mode
        // PDEN           1    Hold in power down while setting up
        0b000_0_0_0_1_1,
    ];

    buf = *control_2_reg;
    defmt::debug!("control_2_reg {:?}", buf);
    match i2c.write(CODEC_ADDR, &buf) {
        Ok(_) => {
            defmt::debug!("Ok");
        }
        Err(err) => {
            i2c_error_to_string!(err);
            panic!("error {:?}", err);
        }
    }

    let init = &[
        0x01u8 | 1 << 7, //   incremental mmap from address 1

        // 01, Mode Control 1
        // M              00   Single Speed
        // R              00   MCLK/LRCK (FS) = 256 -> 4MHz/256 = 15.6kHz
        // M/S            1    Slave Mode
        // DAC_DIF        001  I2S, up to 24 bit
        0b00_00_0_001,
        
        // 02, DAC Control
        // AMUTE          0    Auto Mute Enable (mute on consecutive 0 or-1)
        // FILT_SEL       0    Fast Roll off
        // DEM            00   Deemphasis disabled
        // RMP_UP         0    Immediate
        // RMP_Down       0    Immediate
        // INV            00   0 phase
        0b0_0_00_0_0_00,
        
        // 03, DAC Volume & Mixing Control
        // Reserved       0
        // B=0            1    Linked
        // Soft           1    Soft ramping of volume
        // ZeroCross      1    Minimize artifacts on volume change
        // ATAPI          1001 L/R
        0b0_1_1_1_1001,

        // 04, DAC A Volume Control
        // MUTE           0    Un-muted
        // VOL            0    Full Volume (no attenuation)
        0b0_0000000,

        // 05, DAC B Volume Control
        // MUTE           0    Un-muted
        // VOL            0    Full Volume (no attenuation)
        0b0_0000000,

        // 06 ADC Control
        // Reserved       0
        // Reserved       0
        // Dither16       0    No dither for 16 bit
        // ADC_DIF        1    I2S
        // MUTE_A         0    Un-Muted
        // MUTE_B         0    Un-Muted
        // HPF A Disable  1    DC Mode
        // HPF B Disable  1    DC Mode
        0b00_0_1_00_11,

        // 07 Mode Control 2
        // Reserved       0
        // Reserved       0
        // Reserved       0
        // LOOP           0    No Loopback
        // MUTE A AND B   0    Individual Mute Signals
        // FREEZE         0    Immediate updates
        // CPEN           1    Enable port mode
        // PDEN           1    Still hold in power down
        0b000_0_0_0_1_1,
    ];

    let buf = *init;
    defmt::debug!("init {:?}", buf);
    match i2c.write(CODEC_ADDR, &buf) {
        Ok(_) => {
            defmt::debug!("Ok");
        }
        Err(err) => {
            i2c_error_to_string!(err);
            panic!("error {:?}", err);
        }
    }

    let start = &[
        0x07u8, // 07 Mode Control 2
        // 07 Mode Control 2
        // Reserved       0
        // Reserved       0
        // Reserved       0
        // LOOP           0    No Loopback
        // MUTE A AND B   0    Individual Mute Signals
        // FREEZE         0    Immediate updates
        // CPEN           1    Enable port mode
        // PDEN           0    Power up
        0b000_0_0_0_1_0,
    ];

    let buf = *start;
        defmt::debug!("boot {:?}", buf);
        match i2c.write(CODEC_ADDR, &buf) {
            Ok(_) => {
                defmt::debug!("Ok");
            }
            Err(err) => {
                i2c_error_to_string!(err);
                panic!("error {:?}", err);
            }
        }
}