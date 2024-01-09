#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;

use embassy_executor::{Executor, task};

use embassy_futures::select::select;
use embassy_sync::{pubsub::{PubSubChannel, publisher, Publisher, Subscriber}, blocking_mutex::raw::NoopRawMutex};
use embassy_time::Timer;
use embedded_hal_async::digital::Wait;
// use embedded_hal::digital::InputPin;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{EspWifiInitFor, initialize, esp_now::{EspNow, EspNowSender, BROADCAST_ADDRESS}};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, IO, timer::TimerGroup, embassy, systimer::SystemTimer, Rng, gpio::{Gpio4, Input, PullUp, Gpio6}};
use protocol::ControlMessage;
use rotary_encoder_hal::Rotary;
use static_cell::make_static;

const MAX_MESSAGE: usize = 10;
const MAX_PUBLISHER: usize = 5;
const MAX_SUBSCRIBER: usize = 5;

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

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);

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
    let (_esp_manager, esp_sender, _esp_receiver) = espnow.split();

    let command_channel: PubSubChannel<NoopRawMutex, ControlMessage, MAX_MESSAGE, MAX_SUBSCRIBER, MAX_PUBLISHER> = PubSubChannel::new();
    let command_channel = make_static!(command_channel);

    executor.run(|spawner| {
        spawner.spawn(encoder(io.pins.gpio4.into_pull_up_input(), io.pins.gpio6.into_pull_up_input(), command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(sender(esp_sender,command_channel.subscriber().unwrap())).unwrap();
    })
}

#[task]
async fn sender(mut esp_sender: EspNowSender<'static>, mut subscriber: Subscriber<'static, NoopRawMutex,ControlMessage,MAX_MESSAGE,MAX_SUBSCRIBER,MAX_PUBLISHER>)->! {
    loop {
        let message = subscriber.next_message_pure().await;
        esp_sender.send_async(&BROADCAST_ADDRESS, &message.to_bytes()).await.unwrap();
    }
}

#[task]
async fn encoder(pin_a: Gpio4<Input<PullUp>>,pin_b: Gpio6<Input<PullUp>>, mut publisher: Publisher<'static,NoopRawMutex,ControlMessage,MAX_MESSAGE, MAX_SUBSCRIBER,MAX_PUBLISHER>) {
    let mut rotary = Rotary::new(pin_a, pin_b);
    let mut count = 0_i32;
    loop {
        let (pin_a,pin_b) = rotary.pins();
        select(pin_a.wait_for_any_edge(),pin_b.wait_for_any_edge()).await;
        let direction = rotary.update().unwrap();
        match direction {
            rotary_encoder_hal::Direction::Clockwise => count+=1,
            rotary_encoder_hal::Direction::CounterClockwise => count-=1,
            rotary_encoder_hal::Direction::None => (),
        }
        publisher.publish(ControlMessage::SteeringPosition(count)).await;
        println!("Count: {}",count);

    }
}
