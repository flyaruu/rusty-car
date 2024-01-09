#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;
use embassy_executor::{Executor, task};

use esp_backtrace as _;
use esp_wifi::{EspWifiInitFor, initialize, esp_now::{EspNow, EspNowReceiver}};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, IO, timer::TimerGroup, embassy, systimer::SystemTimer, Rng};

use log::info;
use protocol::ControlMessage;
use static_cell::make_static;



use esp_backtrace as _;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();
    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}


#[entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    // let mut delay = Delay::new(&clocks);

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");

    let _io = IO::new(peripherals.GPIO,peripherals.IO_MUX);

    hal::interrupt::enable(hal::peripherals::Interrupt::GPIO, hal::interrupt::Priority::Priority1).unwrap();
    let executor = make_static!(Executor::new());
    let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks);    
    embassy::init(&clocks,timer_group.timer0);

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

    let espnow = EspNow::new(&init, wifi).unwrap();
    let (_esp_manager, _esp_sender, esp_receiver) = espnow.split();

    executor.run(|spawner| {
        spawner.spawn(receive(esp_receiver)).unwrap();
    })
}

#[task]
async fn receive(mut receiver: EspNowReceiver<'static>) {
    info!("Receiver started");
    loop {
        let msg = receiver.receive_async().await;
        let control_message = ControlMessage::from_slice(msg.get_data());
        info!("Control message: {:?}",control_message);

    }
}