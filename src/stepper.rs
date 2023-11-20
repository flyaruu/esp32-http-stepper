
use core::{convert::Infallible, time::Duration};

use accel_stepper::{Device, SystemClock};
use embedded_hal::digital::{OutputPin, ToggleableOutputPin};
use hal::{gpio::{Gpio1, Output, PushPull, Gpio2}, Rtc};


pub struct StepperClock {
    pub rtc: Rtc<'static>,
}

impl SystemClock for StepperClock {
    fn elapsed(&self) -> core::time::Duration {
        Duration::from_micros(self.rtc.get_time_us())
    }
}

pub struct Stepper {
    pub dir: Gpio1<Output<PushPull>>,
    pub step_pin: Gpio2<Output<PushPull>>,
}

impl Device for Stepper {
    type Error = Infallible;

    fn step(&mut self, ctx: &accel_stepper::StepContext) -> Result<(), Self::Error> {
        if ctx.position < ctx.target_position {
            self.dir.set_high().unwrap();
        } else {
            self.dir.set_low().unwrap();
        }
        self.step_pin.toggle().unwrap();
        Ok(())
    }
}