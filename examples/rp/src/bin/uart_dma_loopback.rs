//! DMA Circular buffered UART receiver test
//!
//! This example demonstrates the DMA circular buffer UART implementation.
//! Send data to PIN_1 (UART RX) to see it received and printed.
//!
//! Implementation details:
//! - RP2350: Uses 1 DMA channel (control_dma ignored, TRIGGER_SELF mode)
//! - RP2040: Uses 2 DMA channels (data + control chaining)

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, UART0};
use embassy_rp::uart::{Config, DmaCircularInterruptHandler, DmaCircularUartRx};
use embedded_io_async::Read;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    UART0_IRQ => DmaCircularInterruptHandler<UART0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("=== DMA Circular UART Loopback Test ===");
    info!("Connect PIN_0 (TX) to PIN_1 (RX)");

    // Create circular RX buffer (must be power of 2)
    static RX_BUFFER: StaticCell<[u8; 512]> = StaticCell::new();
    let rx_buffer = &mut RX_BUFFER.init([0; 512])[..];
    let buffer_size = rx_buffer.len();

    // Create DMA circular RX
    let mut uart_rx = DmaCircularUartRx::new_circular(
        p.UART0,
        p.PIN_1, // RX pin
        Irqs,
        p.DMA_CH0, // Data DMA channel
        p.DMA_CH1, // Control DMA channel (only needed on RP2040)
        rx_buffer,
        Config::default(),
    );

    info!("DMA circular UART initialized");
    info!("Buffer size: {} bytes", buffer_size);
    info!("Waiting for data...");

    // Simple receiver loop in main task
    loop {
        let mut buf = [0u8; 64];

        match uart_rx.read(&mut buf).await {
            Ok(n) => {
                if n > 0 {
                    // Try to interpret as UTF-8
                    match core::str::from_utf8(&buf[..n]) {
                        Ok(s) => info!("RX: {} bytes: {}", n, s.trim()),
                        Err(_) => info!("RX: {} bytes (binary): {:02x}", n, &buf[..n.min(16)]),
                    }
                }
            }
            Err(e) => {
                error!("UART error: {:?}", e);
                // Continue despite errors
            }
        }
    }
}

