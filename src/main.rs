#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use core::{mem::MaybeUninit, cell::RefCell};
use accel_stepper::Driver;
use alloc::rc::Rc;
use embassy_executor::Executor;
use embassy_net::{Config, Stack, StackResources};
use embassy_sync::{channel::{Channel, Receiver, Sender}, blocking_mutex::raw::NoopRawMutex};
use embassy_time::{Timer, Duration};
use embedded_hal::digital::{OutputPin, ToggleableOutputPin};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{EspWifiInitFor, initialize, wifi::WifiStaDevice};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, IO, timer::TimerGroup, embassy, systimer::SystemTimer, Rng, gpio::{PushPull, Gpio2, Gpio1, Output}, Rtc};

use picoserve::{Router, routing::get, response::IntoResponse, extract::{State, Form, FromRequest, FormRejection}};


use serde::Deserialize;
use static_cell::{StaticCell, make_static};


use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp_backtrace as _;
use stepper::{Stepper, StepperClock};

use crate::net::{connection, net_task, web_task};
// use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};

mod stepper;
mod net;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

// static CHANNEL: Channel<CriticalSectionRawMutex,MoveCommand,5> = Channel::new();

const QUEUE_SIZE: usize = 10;

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

struct EmbassyTimer;

impl picoserve::Timer for EmbassyTimer {
    type Duration = embassy_time::Duration;
    type TimeoutError = embassy_time::TimeoutError;

    async fn run_with_timeout<F: core::future::Future>(
        &mut self,
        duration: Self::Duration,
        future: F,
    ) -> Result<F::Output, Self::TimeoutError> {
        embassy_time::with_timeout(duration, future).await
    }
}


#[derive(Clone,Debug,Deserialize)]
struct MoveCommand {
    distance: i32,
    speed: u32,
    accel: u32,
}

struct AppState {
    sender: Sender<'static, NoopRawMutex,MoveCommand,QUEUE_SIZE>,
}


#[entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    // let mut delay = Delay::new(&clocks);
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");
    println!("Hello world!");

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);

    let pin1 = io.pins.gpio1.into_push_pull_output();
    let pin2 = io.pins.gpio2.into_push_pull_output();


    hal::interrupt::enable(hal::peripherals::Interrupt::GPIO, hal::interrupt::Priority::Priority1).unwrap();

    let executor = EXECUTOR.init(Executor::new());

    let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks);    
    embassy::init(&clocks,timer_group.timer0);
    // accel_stepper::Device

    let rtc = Rtc::new(peripherals.RTC_CNTL);

    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();
    let config = Config::dhcpv4(Default::default());

    let seed = 1234; // very random, very secure seed

    // Init network stack
    let stack = &*make_static!(Stack::new(
        wifi_interface,
        config,
        make_static!(StackResources::<3>::new()),
        seed
    ));

    let motor_channel: &'static mut Channel<NoopRawMutex, MoveCommand, QUEUE_SIZE> = make_static!(Channel::new());

    let pico_config = make_static!(picoserve::Config {
        start_read_request_timeout: Some(Duration::from_secs(5)),
        read_request_timeout: Some(Duration::from_secs(1)),
    });

    let app_state = Rc::new(RefCell::new(AppState{ sender: motor_channel.sender()}));

    executor.run(|spawner| {
        spawner.spawn(connection(controller)).unwrap();
        spawner.spawn(net_task(stack)).unwrap();
        spawner.spawn(web_task(stack,pico_config, app_state)).unwrap();
        spawner.spawn(motor_1(pin1, pin2, motor_channel.receiver(),rtc)).unwrap();
    })
}

async fn get_root(Form(payload): Form<MoveCommand>, State(app_state): State<Rc<RefCell<AppState>>>)-> impl IntoResponse {
    app_state.borrow().sender.send( payload).await;
    "command sent!"
}

#[embassy_executor::task]
async fn motor_1(dir: Gpio1<Output<PushPull>>, step: Gpio2<Output<PushPull>>, receiver: Receiver<'static, NoopRawMutex,MoveCommand,QUEUE_SIZE>, rtc: Rtc<'static>) {
    motor(dir, step, receiver, rtc).await
}


async fn motor<D: OutputPin, S: ToggleableOutputPin>(dir: D, step: S, receiver: Receiver<'static, NoopRawMutex,MoveCommand,QUEUE_SIZE>, rtc: Rtc<'static>) {
    let mut stepper = Stepper { dir, step_pin: step };
    let mut driver = Driver::default();
    let clock = StepperClock { rtc };
    loop {
        let command = receiver.receive().await;
        driver.set_acceleration(command.accel as f32);
        driver.set_max_speed(command.speed as f32);
        driver.move_by(command.distance as i64);
        while driver.distance_to_go().abs() > 0  {
            driver.poll(&mut stepper, &clock).unwrap();
            Timer::after(Duration::from_micros(10)).await;
        }

    }
}

