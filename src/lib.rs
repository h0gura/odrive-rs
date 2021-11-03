#![no_std]

pub mod rcc;
pub mod spi;
pub mod as5048a;
pub mod motor;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}