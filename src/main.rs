#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::{asm::wfi, interrupt::Mutex};
use cortex_m_rt::entry;

use embedded_hal::spi::MODE_1;

use stm32f4xx_hal as hal;
use hal::{
    adc::config::{Align, Clock, Continuous, Resolution, Scan},
    gpio::{gpioa, Output, PushPull, gpioc::{PC10, PC11, PC12}, Alternate},
    pac,
    pac::{ADC1, ADC_COMMON, interrupt, Interrupt, TIM1, TIM2},
    prelude::*,
    pwm,
    signature::VDDA_CALIB,
    time::KiloHertz,
    timer::{Event, Timer},
};

use odrive_rs::spi::Spi;
extern crate drv8301;
use drv8301::drv8301::Drv8301;
use odrive_rs::as5048a::AS5048A;
use odrive_rs::motor::Motor;

use odrive_rs::rcc::{Enable, Reset};

use cortex_m_semihosting::{hprint, hprintln};
use panic_halt as _;


//type TypeLed = gpioa::PA2<Output<PushPull>>;
//static G_LED: Mutex<RefCell<Option<TypeLed>>> = Mutex::new(RefCell::new(None));

type TypeSpi3 = Spi<pac::SPI3, (PC10<Alternate<stm32f4xx_hal::gpio::AF6>>, PC11<Alternate<stm32f4xx_hal::gpio::AF6>>, PC12<Alternate<stm32f4xx_hal::gpio::AF6>>)>;
static G_SPI3: Mutex<RefCell<Option<TypeSpi3>>> = Mutex::new(RefCell::new(None));

type TypeEncoder<'a> = AS5048A<'a, TypeSpi3, gpioa::PA3<Output<PushPull>>>;
//static G_AS5048A: Mutex<RefCell<Option<TypeEncoder>>> = Mutex::new(RefCell::new(None));

type TypeMotor = Motor;
static G_MOTOR: Mutex<RefCell<Option<TypeMotor>>> = Mutex::new(RefCell::new(None));

static G_TIM: Mutex<RefCell<Option<Timer<TIM2>>>> = Mutex::new(RefCell::new(None));

static mut CURRENT_A: f32 = 0.0;
static mut CURRENT_B: f32 = 0.0;
static mut CURRENT_C: f32 = 0.0;


#[interrupt]
fn ADC() {
    // current sensing
    unsafe {
        let max_sample:u32 = (1 << 12) - 1;

        let device = pac::Peripherals::steal();
        device.ADC1.sr.modify(|_, w| w.jeoc().clear_bit());

        let sample = device.ADC1.jdr1.read().jdata().bits();
        let so1 = ( ( (u32::from(sample) + 48u32) * VDDA_CALIB ) / max_sample) as u16;

        let sample = device.ADC1.jdr2.read().jdata().bits();
        let so2 = ( ( (u32::from(sample) + 118u32) * VDDA_CALIB ) / max_sample) as u16;

        CURRENT_B = (so1 as f32 - 1650.0) / 200.0;
        CURRENT_C = (so2 as f32 - 1650.0) / 200.0;
        CURRENT_A = - CURRENT_B - CURRENT_C;
    }

    /*
    // LED Debug
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = G_LED.borrow(cs).borrow_mut().as_mut() {
            led.toggle().unwrap();
        }
    });
    */
}

#[interrupt]
fn TIM2() {

    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut tim) = G_TIM.borrow(cs).borrow_mut().as_mut() {
            let _ = tim.wait();
        }
    });

    static mut SPI3: Option<TypeSpi3> = None;
    static mut MOTOR: Option<TypeMotor> = None;
    unsafe{
        let mut spi3 = SPI3.get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| {
                G_SPI3.borrow(cs).replace(None).unwrap()
            })
        });

        let device = pac::Peripherals::steal();
        let gpioa = device.GPIOA.split();
        let ncs = gpioa.pa3.into_push_pull_output();
        let mut as5048: TypeEncoder = AS5048A::new(&mut spi3, ncs);

        // AS5048A
        let angle = as5048.angle().unwrap();
        let offset = 650u16;
        let angle_hosei = (angle - offset) % 16384u16;
        let electric_angle = angle_hosei % (16384/12u16);

        let motor = MOTOR.get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| {
                G_MOTOR.borrow(cs).replace(None).unwrap()
            })
        });

        // select control mode
        //motor.drive_profile().unwrap();
        //motor.drive_sixstep().unwrap();
        //motor.drive_anglebased_sixstep(electric_angle).unwrap();
        motor.drive_foc(electric_angle, CURRENT_A, CURRENT_B, CURRENT_C).unwrap();
    }

    /*
    // LED Debug
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = G_LED.borrow(cs).borrow_mut().as_mut() {
            led.toggle().unwrap();
        }
    });
    */
}


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(168.mhz())
        .hclk(168.mhz())
        .pclk1(42.mhz())
        .pclk2(84.mhz())
        .require_pll48clk()
        .freeze();
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // SPI3
    let sck = gpioc.pc10.into_alternate_af6();
    let miso = gpioc.pc11.into_alternate_af6();
    let mosi = gpioc.pc12.into_alternate_af6();
    let mut spi = Spi::spi3(
        dp.SPI3,
        (sck, miso, mosi),
        MODE_1,
        KiloHertz(2000).into(),
        clocks,
    );

    // DRV8301
    let ncs = gpioc.pc13.into_push_pull_output();
    let en_gate = gpiob.pb12.into_push_pull_output();
    let mut drv8301 = Drv8301::new(&mut spi, ncs, en_gate);
    drv8301.init().unwrap();
    // Move the pin into our global storage
    cortex_m::interrupt::free(|cs| *G_SPI3.borrow(cs).borrow_mut() = Some(spi));
 
    // PWM
    let channels = (gpioa.pa8.into_alternate_af1(), gpioa.pa9.into_alternate_af1(), gpioa.pa10.into_alternate_af1(), gpioa.pa11.into_alternate_af1());
    let pwm = pwm::tim1(dp.TIM1, channels, clocks, 16u32.khz());
    let (ch1, ch2, ch3, ch4) = pwm;
    let mut ch4 = ch4;
    {
        // Set complementary oututs mode as AF1
        gpiob.pb13.into_alternate_af1();
        gpiob.pb14.into_alternate_af1();
        gpiob.pb15.into_alternate_af1();
        unsafe {
            let tim1_regb = &(*(TIM1::ptr()));

            // Enable complementary outputs
            tim1_regb.ccer.modify(|_, w| w.cc1ne().set_bit());
            tim1_regb.ccer.modify(|_, w| w.cc2ne().set_bit());
            tim1_regb.ccer.modify(|_, w| w.cc3ne().set_bit());

            // Set dead time
            tim1_regb.bdtr.modify(|_, w| w.dtg().bits(10));

            // Center aligned
            tim1_regb.cr1.modify(|_, w| w.cms().center_aligned1());

            // OC4REF signal is used as trigger output
            tim1_regb.cr2.modify(|_, w| w.mms().compare_oc4());
        }
        ch4.enable();
        ch4.set_duty( (ch4.get_max_duty() as f32 * 0.99)as u16 );
    }
    delay.delay_ms(1u32);

    // Motor
    let mut motor = Motor::new(ch1, ch2, ch3);
    motor.set_duty(0,0,0).unwrap();
    motor.enable().unwrap();
    delay.delay_ms(1u32);

    /*
    // for current sensing test
    unsafe{
        motor.set_hiz_c();
        motor.set_duty((motor.max_duty as f32 * 0.6) as u16, (motor.max_duty as f32 * 0.4) as u16, 0u16).unwrap();
    }
    */
    cortex_m::interrupt::free(|cs| *G_MOTOR.borrow(cs).borrow_mut() = Some(motor));


    // ADC1
    gpioc.pc0.into_analog();
    gpioc.pc1.into_analog();
    unsafe {
        // All ADCs share the same reset interface.
        // NOTE(unsafe) this reference will only be used for atomic writes with no side effects.
        let rcc = &(*pac::RCC::ptr());

        // Enable the clock
        pac::ADC1::enable(rcc);
        pac::ADC1::reset(rcc);

        let adcc_regb = &(*(ADC_COMMON::ptr()));
        let adc1_regb = &(*(ADC1::ptr()));

        // Probably unnecessary to disable the ADC in most cases but it shouldn't do any harm either
        adc1_regb.cr2.modify(|_, w| w.adon().clear_bit());

        // Config common
        adcc_regb.ccr.modify(|_, w| w.adcpre().bits(Clock::Pclk2_div_2.into()));

        // Config regular conversion
        adc1_regb.cr1.modify(|_, w| w.res().bits(Resolution::Twelve.into()));   //12bit
        adc1_regb.cr1.modify(|_, w| w.scan().bit(Scan::Enabled.into()));
        adc1_regb.cr2.modify(|_, w| w.align().bit(Align::Right.into()));
        adc1_regb.cr2.modify(|_, w| w.cont().bit(Continuous::Single.into()));
        
        // config injected conversion
        adc1_regb.cr1.modify(|_, w | w.jeocie().enabled());
        adc1_regb.cr2.modify(|_, w| w.jexten().rising_edge());
        adc1_regb.cr2.modify(|_, w| w.jextsel().tim1cc4());
        adc1_regb.jsqr.modify(|_, w| w.jl().bits(0b01));
        adc1_regb.jsqr.modify(|_, w| w.jsq3().bits(10u8));
        adc1_regb.jsqr.modify(|_, w| w.jsq4().bits(11u8));
        adc1_regb.smpr1.modify(|_, w| w.smp10().cycles3());
        adc1_regb.smpr1.modify(|_, w| w.smp11().cycles3());
        
        // enable ADC
        adc1_regb.cr2.modify(|_, w| w.adon().set_bit());
        delay.delay_ms(1u32);

        // enable interrupt
        cortex_m::peripheral::NVIC::unmask(Interrupt::ADC);
    }

    /*
    // Debug LED
    let mut led = gpioa.pa2.into_push_pull_output();
    let _ = led.set_high();
    cortex_m::interrupt::free(|cs| *G_LED.borrow(cs).borrow_mut() = Some(led));
    */

    // TIM2 Interrupt
    let mut timer = Timer::tim2(dp.TIM2, 5000.hz(), clocks);
    timer.listen(Event::TimeOut);
    cortex_m::interrupt::free(|cs| *G_TIM.borrow(cs).borrow_mut() = Some(timer));
    //enable TIM2 interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }

    loop {
        wfi();

        /*
        unsafe{
            //hprintln!("CURRENT_A: {}A, CURRENT_B: {}A, CURRENT_C: {}A", CURRENT_A, CURRENT_B, CURRENT_C);
        }
        delay.delay_ms(1u32);
        */
    }
}

