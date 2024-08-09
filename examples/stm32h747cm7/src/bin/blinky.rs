#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{i2c, pac, SharedData};
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Blocking;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

const PMIC_ADDRESS: u8 = 0x08;
const PMIC_SETUP: &[&[u8]] = &[
    &[0x4F, 0x00],
    &[0x50, 0x0F],
    &[0x4C, 0x05],
    &[0x4D, 0x03],
    &[0x52, 0x09],
    &[0x53, 0x0F],
    &[0x9C, 0x80],
    &[0x9E, 0x20],
    &[0x42, 0x02],
    &[0x94, 0xA0],
    &[0x3B, 0x0F],
    &[0x35, 0x0F],
    &[0x42, 0x01],
];

#[link_section = ".ram_d3"]
static SHARED_DATA: MaybeUninit<SharedData> = MaybeUninit::uninit();

async fn pmic_init(mut i2c: I2c<'static, Blocking>) -> Result<(), i2c::Error> {
    info!("PMIC Initialization");
    let address = 8 << 1;
    let mut data = [0u8; 2];

    // LDO2 to 1.8V
    trace!("LDO2 to 1.8V");
    data[0] = 0x4F;
    data[1] = 0x0;
    i2c.blocking_write(address, &data)?;
    data[0] = 0x50;
    data[1] = 0xF;
    i2c.blocking_write(address, &data)?;

    // LDO1 to 1.0V
    trace!("LDO1 to 1.0V");
    data[0] = 0x4C;
    data[1] = 0x5;
    i2c.blocking_write(address, &data)?;
    data[0] = 0x4D;
    data[1] = 0x3;
    i2c.blocking_write(address, &data)?;

    // LDO3 to 1.2V
    trace!("LDO3 to 1.2V");
    data[0] = 0x52;
    data[1] = 0x9;
    i2c.blocking_write(address, &data)?;
    data[0] = 0x53;
    data[1] = 0xF;
    i2c.blocking_write(address, &data)?;

    Timer::after(Duration::from_millis(10)).await;

    data[0] = 0x9C;
    data[1] = 1 << 7;
    i2c.blocking_write(address, &data)?;

    // Disable charger LED
    data[0] = 0x9E;
    data[1] = 1 << 5;
    i2c.blocking_write(address, &data)?;

    Timer::after(Duration::from_millis(10)).await;

    // Set 2A current limit for SW3
    trace!("2A CURRENT LIMIT for SW3");
    data[0] = 0x42;
    data[1] = 2;
    i2c.blocking_write(address, &data)?;

    Timer::after(Duration::from_millis(10)).await;

    // Change VBUS Input Current Limit to 1.5A
    trace!("1.5A INPUT CURRENT LIMIT for VBUS");
    data[0] = 0x94;
    data[1] = 20 << 3;
    i2c.blocking_write(address, &data)?;

    // SW2 to 3.3V
    trace!("SW2 to 3.3V");
    data[0] = 0x3B;
    data[1] = 0xF;
    i2c.blocking_write(address, &data)?;

    // SW1 to 3.3V
    trace!("SW1 to 3.3V");
    data[0] = 0x35;
    data[1] = 0xF;
    i2c.blocking_write(address, &data)?;

    Ok(())
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::time::Hertz;
        use embassy_stm32::rcc::*;
        use embassy_stm32::rcc::mux::*;

        // Common configuration
        config.rcc.hse = Some(Hse {
            freq: Hertz(25_000_000),
            mode: HseMode::Oscillator,
        });

        // PLL1 configuration (for CM7 core)
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV5,
            mul: PllMul::MUL160, // Results in 800 MHz
            divp: Some(PllDiv::DIV2), // 400 MHz
            divq: Some(PllDiv::DIV4), // 200 MHz
            divr: Some(PllDiv::DIV2), // 400 MHz
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 MHz

        // PLL2 configuration (for CM4 core)
        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV5,
            mul: PllMul::MUL96, // Results in 480 MHz
            divp: Some(PllDiv::DIV2), // 240 MHz
            divq: Some(PllDiv::DIV4), // 120 MHz
            divr: Some(PllDiv::DIV2), // 240 MHz
        });

        // AHB and APB prescalers
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 MHz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 MHz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 MHz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 MHz
        config.rcc.apb4_pre = APBPrescaler::DIV4; // 50 MHz

        // CM7 core clock mux
        config.rcc.mux.lptim1sel = Lptim1sel::PCLK1; // 100 MHz

        // CM4 core clock mux
        config.rcc.mux.lptim2sel = Lptim2sel::PCLK4; // 100 MHz

        // Other settings
        config.rcc.voltage_scale = VoltageScale::Scale0;
        config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true });
    }

    let p = embassy_stm32::init_primary(config, &SHARED_DATA);
    // let mut i2c = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Hertz(100_000), Default::default());
    // for &data in PMIC_SETUP {
    //     let mut ret = [0u8; 1];
    //     i2c.blocking_write_read(PMIC_ADDRESS, data, &mut ret).unwrap();
    //     info!("0x{:X}", ret);
    // }
    // pmic_init(i2c).await.expect("Failed PMIC init");

    cortex_m::Peripherals::take().unwrap().SCB.enable_icache();
    pac::RCC.gcr().modify(|w| w.set_boot_c2(true));
    let addr = core::ptr::addr_of!(SHARED_DATA) as usize;
    warn!("shared data at 0x{:X}", addr);
    info!("Hello Primary World!");

    let mut led_red = Output::new(p.PK5, Level::High, Speed::Low);
    let _led_green = Output::new(p.PK6, Level::High, Speed::Low);
    Timer::after(Duration::from_millis(500)).await;

    loop {
        info!("red high");
        led_red.set_high();
        Timer::after_millis(500).await;

        info!("red low");
        led_red.set_low();
        Timer::after_millis(500).await;
    }
}
