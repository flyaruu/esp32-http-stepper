use accel_stepper::{Device, SystemClock};

use embedded_hal::digital::{OutputPin, ToggleableOutputPin};
use hal::Rtc;

pub struct Stepper<D, S> {
    dir_pin: D,
    step_pin: S,
}

impl <D,S> Stepper<D, S> {
    pub fn new(dir_pin: D, step_pin: S)->Self {
        Stepper { dir_pin, step_pin }
    }
}
// Implement the Device trait on our stepper struct to drive a bipolar-style stepper motor:
impl <D: OutputPin, S: ToggleableOutputPin> Device for Stepper<D, S> {
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

// We need an implementation of the SystemClock trait, that will control the accel stepper
// The only thing it needs is a measurement of time-since-beginning, so it can measure time
// between events
impl <'a>SystemClock for EspRtc<'a> {
    fn elapsed(&self) -> core::time::Duration {
        core::time::Duration::from_micros(self.rtc.get_time_us())
    }
}