#![no_std]
#![no_main]

use core::mem::MaybeUninit;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{pac, SharedData};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

const SYSCFG_UR3_BCM4_ADD0_POS: u32 = 16;
const CM4_BOOT_ADDR: u32 = 0x08100000;
const CM4_BOOT_ADDR_SH: u16 = (CM4_BOOT_ADDR >> SYSCFG_UR3_BCM4_ADD0_POS) as u16;

#[link_section = ".ram_d3"]
static SHARED_DATA: MaybeUninit<SharedData> = MaybeUninit::uninit();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    /*    {
            use embassy_stm32::rcc::*;
            config.rcc.hse = Some(Hse {
                freq: Hertz(25_000_000),
                mode: HseMode::Oscillator,
            });
            config.rcc.pll1 = Some(Pll {
                source: PllSource::HSE,
                prediv: PllPreDiv::DIV5,
                mul: PllMul::MUL160,  // Reduced from 192 to 160
                divp: Some(PllDiv::DIV2),
                divq: Some(PllDiv::DIV4),
                divr: Some(PllDiv::DIV2),
            });
            config.rcc.sys = Sysclk::PLL1_P;
            config.rcc.ahb_pre = AHBPrescaler::DIV2;
            config.rcc.apb1_pre = APBPrescaler::DIV2;
            config.rcc.apb2_pre = APBPrescaler::DIV2;
            config.rcc.apb3_pre = APBPrescaler::DIV2;
            config.rcc.apb4_pre = APBPrescaler::DIV4;
            config.rcc.voltage_scale = VoltageScale::Scale1;
            config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true})
        }
    */
    let p = embassy_stm32::init_primary(config, &SHARED_DATA);
    Timer::after(Duration::from_millis(10)).await;
    info!("Hello Primary World!");

    let mut led_red = Output::new(p.PK5, Level::High, Speed::Low);
    let led_green = Output::new(p.PK6, Level::High, Speed::Low);
    pac::SYSCFG.ur3().modify(|w| w.set_boot_add1(CM4_BOOT_ADDR_SH));
    pac::RCC.gcr().modify(|w| w.set_boot_c2(true));

    loop {
        info!("red high");
        led_red.set_high();
        Timer::after_millis(500).await;

        info!("red low");
        led_red.set_low();
        Timer::after_millis(500).await;
    }
}
