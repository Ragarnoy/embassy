//! Test example for DMA circular buffered UART on RP2350
//!
//! This demonstrates self-chaining DMA for continuous UART reception.

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::UART0;
use embassy_rp::uart::{Config, DmaCircularUartRx, InterruptHandler};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandler<UART0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("Starting DMA circular UART test...");

    // Create a circular buffer (must be power of 2 for optimal ring operation)
    static RX_BUFFER: StaticCell<[u8; 512]> = StaticCell::new();
    let rx_buffer = &mut RX_BUFFER.init([0; 512])[..];

    // NOTE: On RP2350, uses TRIGGER_SELF mode (single channel)
    // On RP2040, requires two channels for chaining
    let mut uart_rx = DmaCircularUartRx::new_circular(
        p.UART0,
        p.PIN_1,  // RX pin
        Irqs,
        p.DMA_CH0, // Data DMA channel
        p.DMA_CH1, // Control DMA channel (only needed on RP2040)
        rx_buffer,
        Config::default(),
    );

    info!("DMA circular UART initialized. Waiting for data...");

    // Continuous read loop
    loop {
        let mut buf = [0u8; 32];

        // This will block until data is available
        match uart_rx.read(&mut buf).await {
            Ok(n) => {
                info!("Received {} bytes: {:?}", n, &buf[..n]);
            }
            Err(e) => {
                error!("UART error: {:?}", e);
            }
        }
    }
}
