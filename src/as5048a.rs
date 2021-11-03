#![allow(non_camel_case_types)]

use core::fmt::Debug;

use embedded_hal as hal;
use hal::spi::FullDuplex;
use hal::digital::v2::StatefulOutputPin;
use core::convert::Infallible;
use nb::block;

#[allow(dead_code)]
#[derive(Copy, Clone)]
enum Register {
    ClearErrorFlag = 0x0001,
    ProgrammingControl = 0x0003,
    OtpRegisterZeroPosHigh = 0x0016,
    OtpRegisterZeroPosLow = 0x0017,
    DiagAgc = 0x3FFD,
    Magnitude = 0x3FFE,
    Angle = 0x3FFF,
}


pub struct AS5048A<'a, SPI, CS>
where
    SPI: FullDuplex<u16>,
    CS: StatefulOutputPin<Error = Infallible>,
{
    spi: &'a mut SPI,
    cs: CS,
}

impl<'a, SPI, CS> AS5048A<'a, SPI, CS>
where
    SPI: FullDuplex<u16>,
    CS: StatefulOutputPin<Error = Infallible>,
{
    pub fn new(spi: &mut SPI, cs: CS) -> AS5048A<SPI, CS> {
        AS5048A { spi: spi, cs: cs }
    }

    pub fn diag_gain(&mut self) -> Result<u16, SPI::Error>
    where
    <SPI as FullDuplex<u16>>::Error: Debug
    {
        match self.read(Register::DiagAgc) {
            Ok(arr) => Ok(arr & 0x0f),
            Err(e) => Err(e),
        }
    }

    pub fn magnitude(&mut self) -> Result<u16, SPI::Error>
    where
    <SPI as FullDuplex<u16>>::Error: Debug
    {
        self.read_u16(Register::Magnitude)
    }

    pub fn angle(&mut self) -> Result<u16, SPI::Error>
    where
    <SPI as FullDuplex<u16>>::Error: Debug
    {
        self.read_u16(Register::Angle)
    }

    fn read_u16(&mut self, reg: Register) -> Result<u16, SPI::Error>
    where
    <SPI as FullDuplex<u16>>::Error: Debug
    {
        match self.read(reg) {
            Ok(y) => {
                //let y = u16::from_be_bytes(arr);
                Ok(y & 0b0011_1111_1111_1111)
            }
            Err(e) => Err(e),
        }
    }

    fn read(&mut self, reg: Register) -> Result<u16, SPI::Error>
    where
    <SPI as FullDuplex<u16>>::Error: Debug
    {
        // send cmd
        let mut cmd: u16 = 0b_0100_0000_0000_0000;
        cmd = cmd | reg as u16;
        cmd = set_parity(cmd);

        self.cs.set_low().unwrap();
        cortex_m::asm::delay(8u32);
        block!(self.spi.send(cmd))?;
        block!(self.spi.read())?;
        self.cs.set_high().unwrap();
        cortex_m::asm::delay(32u32);


        // send nop to get result back
        let nop = 0b_0000_0000_0000_0000;
        self.cs.set_low().unwrap();
        cortex_m::asm::delay(8u32);
        block!(self.spi.send(nop))?;
        let ret = block!(self.spi.read())?;
        self.cs.set_high().unwrap();
        cortex_m::asm::delay(32u32);

        Ok(ret)
    }
}

fn set_parity(par: u16) -> u16 {
    let mut x = par;

    x = (x & 0x00FF) ^ (x >> 8);
    x = (x & 0x000F) ^ (x >> 4);
    x = (x & 0x0003) ^ (x >> 2);
    x = (x & 0x0001) ^ (x >> 1);

    if x == 0x0001 {
        par | 0b1000_0000_0000_0000
    } else {
        par
    }
}
