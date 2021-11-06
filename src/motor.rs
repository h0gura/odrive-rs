
use embedded_hal::PwmPin;

use stm32f4xx_hal as hal;
use hal::{
    pac::TIM1,
    pwm::{C1, C2, C3, PwmChannels},
};

use libm;

const SINWAVE: [i32; 192] = [
    17, 15, 13, 12, 10,  8,  7,  6,  5,  4,  3,  2,  1,  1,  1,  0,
    0,  0,  1,  1,  1,  2,  3,  4,  5,  6,  7,  8, 10, 12, 13, 15,
    17, 19, 22, 24, 27, 29, 32, 35, 37, 40, 44, 47, 50, 53, 57, 60,
    64, 67, 71, 75, 79, 82, 86, 90, 94, 98,102,106,110,115,119,123,
    127,131,135,139,144,148,152,156,160,164,168,172,175,179,183,187,
    190,194,197,201,204,207,210,214,217,219,222,225,227,230,232,235,
    237,239,241,242,244,246,247,248,249,250,251,252,253,253,253,254,
    254,254,253,253,253,252,251,250,249,248,247,246,244,242,241,239,
    237,235,232,230,227,225,222,219,217,214,210,207,204,201,197,194,
    190,187,183,179,175,172,168,164,160,156,152,148,144,139,135,131,
    127,123,119,115,110,106,102, 98, 94, 90, 86, 82, 79, 75, 71, 67,
    64, 60, 57, 53, 50, 47, 44, 40, 37, 35, 32, 29, 27, 24, 22, 19
];
static mut IDX: usize = 0;

#[derive(Debug)]
enum Step {
    Step1AtoB,
    Step2AtoC,
    Step3BtoC,
    Step4BtoA,
    Step5CtoA,
    Step6CtoB,
}
static mut STEP: Step = Step::Step1AtoB;

static mut ERR_D_INT: f32 = 0.0;
static mut ERR_Q_INT: f32 = 0.0;


pub struct Motor
{
    ch1: PwmChannels<TIM1, C1>,
    ch2: PwmChannels<TIM1, C2>,
    ch3: PwmChannels<TIM1, C3>,
    pub max_duty: u16,
}

impl Motor
{
    pub fn new(ch1: PwmChannels<TIM1, C1>, ch2: PwmChannels<TIM1, C2>, ch3: PwmChannels<TIM1, C3>) -> Motor {
        let max_duty = ch1.get_max_duty();

        Motor{ 
            ch1, ch2, ch3, max_duty,
        }
    }

    pub fn enable(&mut self) -> Result<(), ()>
    {
        self.ch1.enable();
        self.ch2.enable();
        self.ch3.enable();

        Ok(())
    }

    pub fn set_duty(&mut self, pwm_duty_a: u16, pwm_duty_b: u16, pwm_duty_c: u16) -> Result<(), ()>
    {
        self.ch1.set_duty(pwm_duty_a);
        self.ch2.set_duty(pwm_duty_b);
        self.ch3.set_duty(pwm_duty_c);

        Ok(())
    }

    pub unsafe fn set_hiz_a(&mut self){
        let tim1_regb = &(*(TIM1::ptr()));
        //A phase
        tim1_regb.ccer.modify(|_, w| w.cc1e().clear_bit());
        tim1_regb.ccer.modify(|_, w| w.cc1ne().clear_bit());
        //B phase
        tim1_regb.ccer.modify(|_, w| w.cc2e().set_bit());
        tim1_regb.ccer.modify(|_, w| w.cc2ne().set_bit());
        //C phase
        tim1_regb.ccer.modify(|_, w| w.cc3e().set_bit());
        tim1_regb.ccer.modify(|_, w| w.cc3ne().set_bit());
    } 
    
    pub unsafe fn set_hiz_b(&mut self){
        let tim1_regb = &(*(TIM1::ptr()));
        //A phase
        tim1_regb.ccer.modify(|_, w| w.cc1e().set_bit());
        tim1_regb.ccer.modify(|_, w| w.cc1ne().set_bit());
        //B phase
        tim1_regb.ccer.modify(|_, w| w.cc2e().clear_bit());
        tim1_regb.ccer.modify(|_, w| w.cc2ne().clear_bit());
        //C phase
        tim1_regb.ccer.modify(|_, w| w.cc3e().set_bit());
        tim1_regb.ccer.modify(|_, w| w.cc3ne().set_bit());
    } 
    
    pub unsafe fn set_hiz_c(&mut self){
        let tim1_regb = &(*(TIM1::ptr()));
        //A phase
        tim1_regb.ccer.modify(|_, w| w.cc1e().set_bit());
        tim1_regb.ccer.modify(|_, w| w.cc1ne().set_bit());
        //B phase
        tim1_regb.ccer.modify(|_, w| w.cc2e().set_bit());
        tim1_regb.ccer.modify(|_, w| w.cc2ne().set_bit());
        //C phase
        tim1_regb.ccer.modify(|_, w| w.cc3e().clear_bit());
        tim1_regb.ccer.modify(|_, w| w.cc3ne().clear_bit());
    } 


    pub unsafe fn drive_profile(&mut self) -> Result<(), ()>
    {
        IDX = (IDX + 1) % 192;
        let step_a = IDX;
        let step_b = (step_a + 64) % 192;
        let step_c = (step_b + 64) % 192;

        let pwm_duty_a = (self.max_duty as f32 * (SINWAVE[step_a] as f32 / 255.0)) as u16;
        let pwm_duty_b = (self.max_duty as f32 * (SINWAVE[step_b] as f32 / 255.0)) as u16;
        let pwm_duty_c = (self.max_duty as f32 * (SINWAVE[step_c] as f32 / 255.0)) as u16;

        self.set_duty(pwm_duty_a, pwm_duty_b, pwm_duty_c)
    }

    pub unsafe fn drive_sixstep(&mut self) -> Result<(), ()>
    {
        let mut pwm_duty_a = 0u16;
        let mut pwm_duty_b = 0u16;
        let mut pwm_duty_c = 0u16;

        let duty_plus = 0.6f32;
        let duty_minus = 0.4f32;

        match STEP {
            Step::Step1AtoB => {
                self.set_hiz_c();
                pwm_duty_a = (self.max_duty as f32 * duty_plus) as u16;
                pwm_duty_b = (self.max_duty as f32 * duty_minus) as u16;
                STEP = Step::Step2AtoC;
            }
            Step::Step2AtoC => {
                self.set_hiz_b();
                pwm_duty_a = (self.max_duty as f32 * duty_plus) as u16;
                pwm_duty_c = (self.max_duty as f32 * duty_minus) as u16;
                STEP = Step::Step3BtoC;
            }
            Step::Step3BtoC => {
                self.set_hiz_a();
                pwm_duty_b = (self.max_duty as f32 * duty_plus) as u16;
                pwm_duty_c = (self.max_duty as f32 * duty_minus) as u16;
                STEP = Step::Step4BtoA;
            }
            Step::Step4BtoA => {
                self.set_hiz_c();
                pwm_duty_b = (self.max_duty as f32 * duty_plus) as u16;
                pwm_duty_a = (self.max_duty as f32 * duty_minus) as u16;
                STEP = Step::Step5CtoA;
            }
            Step::Step5CtoA => {
                self.set_hiz_b();
                pwm_duty_c = (self.max_duty as f32 * duty_plus) as u16;
                pwm_duty_a = (self.max_duty as f32 * duty_minus) as u16;
                STEP = Step::Step6CtoB;
            }
            Step::Step6CtoB => {
                self.set_hiz_a();
                pwm_duty_c = (self.max_duty as f32 * duty_plus) as u16;
                pwm_duty_b = (self.max_duty as f32 * duty_minus) as u16;
                STEP = Step::Step1AtoB; 
            }
        }

        self.set_duty(pwm_duty_a, pwm_duty_b, pwm_duty_c)
    }

    pub unsafe fn drive_anglebased_sixstep(&mut self, angle: u16) -> Result<(), ()>
    {
        let mut pwm_duty_a = 0u16;
        let mut pwm_duty_b = 0u16;
        let mut pwm_duty_c = 0u16;

        let duty = 0.1f32;

        if angle < 114 {
            STEP = Step::Step3BtoC;
        }
        else if angle < 342 {
            STEP = Step::Step4BtoA;
        }
        else if angle < 569 {
            STEP = Step::Step5CtoA;
        }
        else if angle < 797 {
            STEP = Step::Step6CtoB;
        }
        else if angle < 1024 {
            STEP = Step::Step1AtoB;
        }
        else if angle < 1252 {
            STEP = Step::Step2AtoC;
        }
        else {
            STEP = Step::Step3BtoC;
        }

        match STEP {
            Step::Step1AtoB => {
                self.set_hiz_c();
                pwm_duty_a = (self.max_duty as f32 * (0.5 + duty)) as u16;
                pwm_duty_b = (self.max_duty as f32 * (0.5 - duty)) as u16;
            }
            Step::Step2AtoC => {
                self.set_hiz_b();
                pwm_duty_a = (self.max_duty as f32 * (0.5 + duty)) as u16;
                pwm_duty_c = (self.max_duty as f32 * (0.5 - duty)) as u16;
            }
            Step::Step3BtoC => {
                self.set_hiz_a();
                pwm_duty_b = (self.max_duty as f32 * (0.5 + duty)) as u16;
                pwm_duty_c = (self.max_duty as f32 * (0.5 - duty)) as u16;
            }
            Step::Step4BtoA => {
                self.set_hiz_c();
                pwm_duty_b = (self.max_duty as f32 * (0.5 + duty)) as u16;
                pwm_duty_a = (self.max_duty as f32 * (0.5 - duty)) as u16;
            }
            Step::Step5CtoA => {
                self.set_hiz_b();
                pwm_duty_c = (self.max_duty as f32 * (0.5 + duty)) as u16;
                pwm_duty_a = (self.max_duty as f32 * (0.5 - duty)) as u16;
            }
            Step::Step6CtoB => {
                self.set_hiz_a();
                pwm_duty_c = (self.max_duty as f32 * (0.5 + duty)) as u16;
                pwm_duty_b = (self.max_duty as f32 * (0.5 - duty)) as u16;
            }
        }

        self.set_duty(pwm_duty_a, pwm_duty_b, pwm_duty_c)
    }

    pub fn drive_foc(&mut self, angle: u16, curr_a: f32, curr_b: f32, curr_c: f32) -> Result<(), ()>
    {
        let curr_alpha: f32 = 0.81694965496580928 * (curr_a - 0.5 * (curr_b + curr_c)); 
        let curr_beta: f32 = 0.7071067811866 * (curr_b - curr_c);

        let th_dc: f32  = 2.0 * 3.1415926353 * angle as f32 / (16384.0/12.0);
        let sin0: f32 = libm::sinf(th_dc);
        let cos0: f32  = libm::cosf(th_dc);

        //αβ -> dq
        let curr_d: f32 = curr_alpha * cos0 + curr_beta * sin0;
        let curr_q: f32 = -curr_alpha * sin0 + curr_beta * cos0;

        let err_d: f32 = 0.0 - curr_d;
        let vol_d: f32;
        unsafe {
            ERR_D_INT += err_d;
            vol_d = 1.0 * err_d + 0.00001 * ERR_D_INT;
        }


        let ref_curr_q: f32 = 0.5;

        let err_q: f32 = ref_curr_q - curr_q;
        let vol_q: f32;
        unsafe {
            ERR_Q_INT += err_q;
            vol_q = 1.0 * err_q + 0.00001 * ERR_Q_INT;
        }


        // ref
        //let vol_d: f32 = 0.0;
        //let vol_q: f32 = 3.0;

        // dq -> αβ
        let vol_alpha: f32 = vol_d * cos0 - vol_q * sin0;
        let vol_beta: f32 = vol_d * sin0 + vol_q * cos0;

        // αβ -> abc
        let mut vol_a: f32 = 0.81649658 * vol_alpha;
        let mut vol_b: f32 = -0.40824829 * vol_alpha + 0.7071067891 * vol_beta;
        let mut vol_c: f32 = -0.40824829 * vol_alpha - 0.7071067891 * vol_beta;

        vol_a = 1.0 - (vol_a + 12.0)/24.0;
        if vol_a > 1.0 { vol_a = 1.0; }
        if vol_a < 0.0 { vol_a = 0.0}

        vol_b = 1.0 - (vol_b + 12.0)/24.0;
        if vol_b > 1.0 { vol_b = 1.0; }
        if vol_b < 0.0 { vol_b = 0.0}

        vol_c = 1.0 - (vol_c + 12.0)/24.0;
        if vol_c > 1.0 { vol_c = 1.0; }
        if vol_c < 0.0 { vol_c = 0.0}

        let pwm_duty_a = (self.max_duty as f32 * (vol_a)) as u16;
        let pwm_duty_b = (self.max_duty as f32 * (vol_b)) as u16;
        let pwm_duty_c = (self.max_duty as f32 * (vol_c)) as u16;

        self.set_duty(pwm_duty_a, pwm_duty_b, pwm_duty_c)
    } 
}

