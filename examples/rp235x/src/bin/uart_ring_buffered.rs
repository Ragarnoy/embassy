//! This example shows how to use the ring-buffered DMA UART RX driver on the RP235x.
//!
//! [`RingBufferedUartRx`] receives continuously into a pool of DMA buffers driven by two
//! chained DMA channels. Reception never stops between reads and there is no per-byte
//! interrupt, so it keeps up at baud rates where the interrupt-driven `BufferedUart`
//! starts dropping bytes.
//!
//! This is a self-test with two phases:
//!
//! - **Capacity**: send a burst that fits in the pool while the receiving task is parked in
//!   the TX await, then read it back and verify every byte. This is what the driver is for:
//!   the DMA captures the whole burst with no CPU involvement.
//! - **Overrun**: deliberately send more than the pool holds while the task is still busy,
//!   and confirm the driver reports `Error::Overrun` rather than returning corrupt data,
//!   then recovers on the next round.
//!
//! **Wiring: connect PIN_4 (UART1 TX) to PIN_1 (UART0 RX) with a jumper wire.**

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, DMA_CH2, UART0};
use embassy_rp::uart::{Config, Error, InterruptHandler, RingBufferedUartRx, UartTx};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::Timer;
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandler<UART0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

/// The RX DMA pool. It is split into 4 segments internally, so a blocked reader is woken
/// every 64 bytes.
const RX_POOL_LEN: usize = 256;

/// Bytes sent in the capacity phase. The driver reports an overrun once the writer comes
/// within one segment of the reader, so usable capacity is the pool minus one segment (192
/// bytes here). Sit comfortably under that rather than exactly on the boundary.
const BURST_LEN: usize = RX_POOL_LEN / 2;

/// Bytes sent in the overrun phase: past the usable capacity, but still inside one pool.
///
/// It has to stay under `RX_POOL_LEN`. The driver infers how far the DMA advanced from the
/// difference between two samples of the write pointer, and that difference is taken modulo
/// the pool size — so a reader starved for a whole pool or more cannot be distinguished from
/// one that is up to date. See the driver's "Limitations" docs.
const FLOOD_LEN: usize = RX_POOL_LEN - RX_POOL_LEN / 16;

/// Content of the stream at byte `i`. Any dropped or duplicated byte shifts the sequence
/// and shows up as a mismatch.
fn pattern(i: usize, round: u32) -> u8 {
    (i as u8).wrapping_mul(31).wrapping_add(round as u8)
}

/// Read until `buf` is full, or return the first error.
async fn read_exact(rx: &mut RingBufferedUartRx<'_>, buf: &mut [u8]) -> Result<(), Error> {
    let mut n = 0;
    while n < buf.len() {
        n += rx.read(&mut buf[n..]).await?;
    }
    Ok(())
}

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut config = Config::default();
    config.baudrate = 1_000_000;

    let mut tx = UartTx::new(p.UART1, p.PIN_4, p.DMA_CH2, Irqs, config);

    static RX_POOL: StaticCell<[u8; RX_POOL_LEN]> = StaticCell::new();
    let rx_pool = &mut RX_POOL.init([0; RX_POOL_LEN])[..];

    let mut rx = RingBufferedUartRx::new(
        p.UART0, p.PIN_1, p.DMA_CH0, // data channel
        p.DMA_CH1, // control channel
        Irqs, rx_pool, config,
    );

    info!("ring-buffered UART RX self-test @ {} baud", config.baudrate);
    info!("jumper PIN_4 (UART1 TX) -> PIN_1 (UART0 RX)");
    info!("pool {} bytes, guaranteed capacity {} bytes", RX_POOL_LEN, BURST_LEN);

    let mut round: u32 = 0;
    let mut failures: u32 = 0;

    loop {
        round += 1;

        // --- Capacity phase -------------------------------------------------------------
        // The whole burst arrives while this task is parked in the TX await, so nothing is
        // polling the receiver. The DMA ring has to capture all of it unaided.
        let mut sent = [0u8; BURST_LEN];
        for (i, b) in sent.iter_mut().enumerate() {
            *b = pattern(i, round);
        }
        unwrap!(tx.write(&sent).await);

        // Let the last byte clear the line before reading.
        Timer::after_millis(5).await;

        let mut got = [0u8; BURST_LEN];
        match read_exact(&mut rx, &mut got).await {
            Ok(()) if got == sent => info!("round {}: capacity OK, {} bytes verified", round, BURST_LEN),
            Ok(()) => {
                failures += 1;
                let first_bad = (0..BURST_LEN).find(|&i| got[i] != sent[i]);
                error!("round {}: MISMATCH at {:?}", round, first_bad);
            }
            Err(e) => {
                failures += 1;
                error!("round {}: capacity phase failed: {}", round, e);
            }
        }

        // --- Overrun phase --------------------------------------------------------------
        // Every fifth round, deliberately overflow the pool and check that the driver says
        // so instead of handing back silently corrupted data.
        if round % 5 == 0 {
            let mut flood = [0u8; FLOOD_LEN];
            for (i, b) in flood.iter_mut().enumerate() {
                *b = pattern(i, round);
            }
            unwrap!(tx.write(&flood).await);
            Timer::after_millis(5).await;

            let mut sink = [0u8; FLOOD_LEN];
            match read_exact(&mut rx, &mut sink).await {
                Err(Error::Overrun) => info!("round {}: overrun correctly reported", round),
                Err(e) => {
                    failures += 1;
                    error!("round {}: expected Overrun, got {}", round, e);
                }
                Ok(()) => {
                    failures += 1;
                    error!("round {}: expected Overrun, but the read succeeded", round);
                }
            }

            // Drain whatever the resync left behind so the next round starts clean.
            let mut scratch = [0u8; RX_POOL_LEN];
            while !rx.is_empty() {
                let _ = rx.blocking_read(&mut scratch);
            }
        }

        if failures > 0 {
            warn!("{} failures across {} rounds", failures, round);
        }

        Timer::after_secs(1).await;
    }
}
