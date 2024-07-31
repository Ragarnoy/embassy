#![no_std]
#![no_main]

use core::mem::MaybeUninit;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::SharedData;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[link_section = ".ram_d3"]
static SHARED_DATA: MaybeUninit<SharedData> = MaybeUninit::uninit();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init_secondary(&SHARED_DATA);
    info!("Hello World!");

    let mut led = Output::new(p.PK7, Level::High, Speed::Low);

    loop {
        info!("blue high");
        led.set_high();
        Timer::after_millis(250).await;

        info!("blue low");
        led.set_low();
        Timer::after_millis(250).await;
    }
}