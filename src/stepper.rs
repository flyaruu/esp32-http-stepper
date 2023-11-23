use accel_stepper::{Device, SystemClock};

use embedded_hal::digital::{OutputPin, ToggleableOutputPin};
use hal::{gpio::{PushPull, Output, Gpio1, Gpio2}, Rtc};

pub struct Stepper {
    dir_pin: Gpio1<Output<PushPull>>,
    step_pin: Gpio2<Output<PushPull>>,
}

impl Stepper {
    pub fn new(dir_pin: Gpio1<Output<PushPull>>, step_pin: Gpio2<Output<PushPull>>)->Self {
        Stepper { dir_pin, step_pin }
    }
}

impl Device for Stepper {
    type Error = ();

    fn step(&mut self, ctx: &accel_stepper::StepContext) -> Result<(), Self::Error> {
        if ctx.position < ctx.target_position {
            self.dir_pin.set_high().unwrap();
        } else {
            self.dir_pin.set_low().unwrap();
        }
        self.step_pin.toggle().unwrap();
        Ok(())
    }
}

#[derive(Clone)]
pub struct EspRtc<'a> {
    rtc: &'a Rtc<'static>
}

impl <'a>EspRtc<'a> {
    pub fn new(rtc: &'a Rtc<'static>)->Self {
        EspRtc { rtc }
    }
}

impl <'a>SystemClock for EspRtc<'a> {
    fn elapsed(&self) -> core::time::Duration {
        core::time::Duration::from_micros(self.rtc.get_time_us())
    }
}