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
    // let shifted_rate = rate.unwrapped_shl(14);
    // Remove the last byte
    let res = shifted_rate.to_be_bytes();
    [res[0], res[1], res[2]]
}

pub fn add(left: usize, right: usize) -> usize {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
