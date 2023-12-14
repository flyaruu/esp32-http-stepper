#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

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
use esp_wifi::{EspWifiInitFor, initialize, wifi::{WifiStaDevice, WifiController, WifiState, WifiEvent, WifiDevice}};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, IO, timer::TimerGroup, embassy, systimer::SystemTimer, Rng, gpio::{PushPull, Gpio2, Gpio1, Output, Gpio6, Gpio7}, Rtc};

use picoserve::{Router, routing::{get, parse_path_segment}, response::IntoResponse, extract::{State, Form}};


use serde::Deserialize;
use static_cell::{StaticCell, make_static};


use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp_backtrace as _;
use stepper::{EspRtc, Stepper};
// use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};


mod stepper;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

// static CHANNEL: Channel<CriticalSectionRawMutex,MoveCommand,5> = Channel::new();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

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

// AppState is the shared state of the HTTP webapp, it can be accessed by passing
// an 'Axum-style' extractor to the handlers.
struct AppState {
    senders: [Sender<'static, NoopRawMutex, MoveCommand, QUEUE_SIZE>; 2]
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

    let pin6 = io.pins.gpio6.into_push_pull_output();
    let pin7 = io.pins.gpio7.into_push_pull_output();


    let rtc: &'static Rtc<'static> = make_static!(Rtc::new(peripherals.RTC_CNTL));

    hal::interrupt::enable(hal::peripherals::Interrupt::GPIO, hal::interrupt::Priority::Priority1).unwrap();

    let executor = EXECUTOR.init(Executor::new());

    let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks);    
    embassy::init(&clocks,timer_group.timer0);
    // accel_stepper::Device

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

    let motor_channel_1: &'static mut Channel<NoopRawMutex, MoveCommand, QUEUE_SIZE> = make_static!(Channel::new());
    let motor_channel_2: &'static mut Channel<NoopRawMutex, MoveCommand, QUEUE_SIZE> = make_static!(Channel::new());

    // Take the two sending sides of the channels, and pass them to the web endpoint
    let motor_senders = [motor_channel_1.sender(),motor_channel_2.sender()];

    let pico_config = make_static!(picoserve::Config {
        start_read_request_timeout: Some(Duration::from_secs(5)),
        read_request_timeout: Some(Duration::from_secs(1)),
    });

    executor.run(|spawner| {
        spawner.spawn(connection(controller)).unwrap();
        spawner.spawn(net_task(stack)).unwrap();
        spawner.spawn(web_task(stack,pico_config, motor_senders)).unwrap();
        spawner.spawn(motor_1(pin1, pin2, motor_channel_1.receiver(),rtc)).unwrap();
        spawner.spawn(motor_2(pin6, pin7, motor_channel_2.receiver(),rtc)).unwrap();
    })
}

// This endpoint will receive commands as an HTTP form
// motor_index: which motor are we driving, extracted from the path
// Form(payload): Destructures the form data into a MoveCommand structure
// State(app_state): Destructures the appstate, which creates the senders
async fn post_command(motor_index: usize, Form(payload): Form<MoveCommand>, State(app_state): State<Rc<RefCell<AppState>>>)-> impl IntoResponse {
    app_state.borrow().senders[motor_index].send(payload).await;
    "command sent!"
}

// Create two instances of a motor task, with different pins.
// embassy tasks can not be generic so we need to use concrete Gpio pin types

#[embassy_executor::task]
async fn motor_1(dir: Gpio1<Output<PushPull>>, step: Gpio2<Output<PushPull>>, receiver: Receiver<'static, NoopRawMutex,MoveCommand,QUEUE_SIZE>, rtc: &'static Rtc<'static>) {
    motor(dir, step, receiver, rtc).await;
}

#[embassy_executor::task]
async fn motor_2(dir: Gpio6<Output<PushPull>>, step: Gpio7<Output<PushPull>>, receiver: Receiver<'static, NoopRawMutex,MoveCommand,QUEUE_SIZE>, rtc: &'static Rtc<'static>) {
    motor(dir, step, receiver, rtc).await;
}


// Take two pins (direction and step) as well as a receiver through which this task will
// receive motor commands. 
async fn motor<D: OutputPin, S: ToggleableOutputPin>(dir: D, step: S, receiver: Receiver<'static, NoopRawMutex,MoveCommand,QUEUE_SIZE>, rtc: &'static Rtc<'static>) {
    let mut driver = Driver::default();


    let clock = EspRtc::new(&rtc);
    let mut device = Stepper::new(dir,step);
    // driver.poll(device, clock);
    loop {
        let command = receiver.receive().await;
        driver.set_max_speed(command.speed as f32);
        driver.set_acceleration(command.accel as f32);            
        driver.move_by(command.distance as i64);
        while driver.is_running() {
            driver.poll(&mut device, &clock).unwrap();          
            Timer::after(Duration::from_micros(10)).await;
        }
        println!("Command complete");
    }
}


#[embassy_executor::task]
async fn web_task(
    stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>,
    config: &'static picoserve::Config<Duration>,
    senders: [Sender<'static, NoopRawMutex, MoveCommand,QUEUE_SIZE>;2]
) -> ! {
    let mut rx_buffer = [0; 1024];
    let mut tx_buffer = [0; 1024];

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    loop {
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        log::info!("Listening on TCP:80...");
        if let Err(e) = socket.accept(80).await {
            log::warn!("accept error: {:?}", e);
            continue;
        }

        log::info!(
            "Received connection from {:?}",
            socket.remote_endpoint()
        );

        let (socket_rx, socket_tx) = socket.split();

        let app = Router::new()
            .route(("/cmd", parse_path_segment::<usize>()), get(post_command))
        ;
        let state = Rc::new(RefCell::new(AppState{senders}));
        match picoserve::serve_with_state(
            &app,
            EmbassyTimer,
            config,
            &mut [0; 2048],
            socket_rx,
            socket_tx,
            &state
        )
        .await
        {
            Ok(handled_requests_count) => {
                log::info!(
                    "{handled_requests_count} requests handled from {:?}",
                    socket.remote_endpoint()
                );
            }
            Err(err) => log::error!("{err:?}"),
        }
    }
}


#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        
        if !matches!(controller.is_ap_enabled(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.into(),
                password: PASSWORD.into(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start().await.unwrap();
            println!("Wifi started!");
        }
        println!("About to connect...");

        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static,WifiStaDevice >>) {
    stack.run().await
}
