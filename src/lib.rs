#![no_std]

pub use usb_device;
pub use usbd_audio;
pub use fixed;

pub type FfCounter = fixed::types::U16F16;

/// The starting value
/// `000 00xx xxxx xxxx. yyyy yyyy yyyy yy00`
/// will be shifted upp by 6 bits and the last byte will be
/// ignored, resulting in
/// `xxxx xxxx xx,yy yyyy yyyy yyyy`
pub fn sample_rate_to_buffer(rate: FfCounter) -> [u8; 3] {
    // Shift value upp by six bytes
    let shifted_rate = rate.unwrapped_shl(6);
    // Remove the last byte
    let res = shifted_rate.to_be_bytes();
    [res[0], res[1], res[2]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rate_formating() {
        let res_buffer = sample_rate_to_buffer(FfCounter::lit("0b0000_0010_0000_0001_1000_0000_0000_0100@-16"));
        let persumed_buffer = fixed::types::U10F22::lit("0b1000_0000_0110_0000_0000_0001@-14").to_be_bytes();
        assert_eq!(res_buffer, [persumed_buffer[0], persumed_buffer[1], persumed_buffer[2]]);
    }
}
