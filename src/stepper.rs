
use core::{convert::Infallible, time::Duration};

use accel_stepper::{Device, SystemClock};
use embedded_hal::digital::{OutputPin, ToggleableOutputPin};
use hal::Rtc;


pub struct StepperClock {
    pub rtc: Rtc<'static>,
}

impl SystemClock for StepperClock {
    fn elapsed(&self) -> core::time::Duration {
        Duration::from_micros(self.rtc.get_time_us())
    }
}

pub struct Stepper<D: OutputPin, S: ToggleableOutputPin> {
    pub dir: D,
    pub step_pin: S,
}

impl<D: OutputPin, S: ToggleableOutputPin>  Device for Stepper<D,S> {
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