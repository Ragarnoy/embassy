//! This example shows how to use the PIO-backed ring-buffered DMA UART TX driver on the RP235x.
//!
//! [`PioRingBufferedUartTx`] copies writes into a circular pool and sends them with one DMA
//! channel straight into a PIO state machine's TX FIFO, restarting it from the DMA interrupt for
//! each contiguous run of queued bytes. There is no per-byte interrupt.
//!
//! The transmitter is checked by receiving what it sends with [`RingBufferedUartRx`]. Each
//! round has three phases:
//!
//! - **Stream**: push several times the pool size through `write_all` while another task
//!   reads, so the pool wraps many times and `write` has to wait for space.
//! - **Odd sizes**: single small writes of 1 to 13 bytes, each flushed and read back. This
//!   exercises starting a run from an idle channel over and over.
//! - **Flush**: once `flush` returns, every byte must already have been received.
//!
//! **Wiring: connect PIN_4 (PIO TX) to PIN_1 (UART0 RX) with a jumper wire.**

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::dma_ring_tx::TxInterruptHandler;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH2, PIO0, UART0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::uart::{PioRingBufferedUartTx, PioUartTxProgram};
use embassy_rp::uart::{Config, Error, InterruptHandler, RingBufferedUartRx};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Instant, Timer};
use embedded_io_async::Write;
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandler<UART0>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    // The TX channel needs its own handler; it cannot share `dma::InterruptHandler`.
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, TxInterruptHandler<DMA_CH2>;
});

const BAUD: u32 = 1_000_000;

/// Deliberately small, so the stream wraps it and waits for space many times.
const TX_POOL_LEN: usize = 64;
const RX_POOL_LEN: usize = 256;

/// Several pools' worth, so the writer has to wait on the DMA.
const STREAM_LEN: usize = 1500;

/// Content of the stream at byte `i`. A dropped, duplicated or reordered byte shows up as a
/// mismatch.
fn pattern(i: usize, round: u32) -> u8 {
    (i as u8).wrapping_mul(31).wrapping_add(round as u8)
}

/// Read until `buf` is full, checking each byte against `pattern`. Returns the offset of the
/// first mismatch, if any.
async fn read_and_check(rx: &mut RingBufferedUartRx<'_>, len: usize, round: u32) -> Result<Option<usize>, Error> {
    let mut first_bad = None;
    let mut got = 0;
    let mut buf = [0u8; 64];
    while got < len {
        let want = (len - got).min(buf.len());
        let n = rx.read(&mut buf[..want]).await?;
        for (j, b) in buf[..n].iter().enumerate() {
            if *b != pattern(got + j, round) && first_bad.is_none() {
                first_bad = Some(got + j);
            }
        }
        got += n;
    }
    Ok(first_bad)
}

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut config = Config::default();
    config.baudrate = BAUD;

    static TX_POOL: StaticCell<[u8; TX_POOL_LEN]> = StaticCell::new();
    let tx_pool = &mut TX_POOL.init([0; TX_POOL_LEN])[..];
    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
    let tx_program = PioUartTxProgram::new(&mut common);
    let mut tx = PioRingBufferedUartTx::new(BAUD, &mut common, sm0, p.PIN_4, &tx_program, p.DMA_CH2, Irqs, tx_pool);

    static RX_POOL: StaticCell<[u8; RX_POOL_LEN]> = StaticCell::new();
    let rx_pool = &mut RX_POOL.init([0; RX_POOL_LEN])[..];
    let mut rx = RingBufferedUartRx::new(p.UART0, p.PIN_1, p.DMA_CH0, p.DMA_CH1, Irqs, rx_pool, config);

    info!("PIO ring-buffered UART TX self-test @ {} baud", BAUD);
    info!("jumper PIN_4 (PIO TX) -> PIN_1 (UART0 RX)");
    info!("tx pool {} bytes, rx pool {} bytes", TX_POOL_LEN, RX_POOL_LEN);

    let mut round: u32 = 0;
    let mut failures: u32 = 0;

    loop {
        round += 1;

        // --- Stream ---------------------------------------------------------------------
        let mut data = [0u8; STREAM_LEN];
        for (i, b) in data.iter_mut().enumerate() {
            *b = pattern(i, round);
        }
        let t0 = Instant::now();
        let (write_res, read_res) = join(
            async {
                let r = tx.write_all(&data).await;
                tx.flush().await;
                r
            },
            read_and_check(&mut rx, STREAM_LEN, round),
        )
        .await;
        let elapsed = t0.elapsed().as_micros();
        unwrap!(write_res);
        // 10 bits per byte on the wire.
        let wire = STREAM_LEN as u64 * 10 * 1_000_000 / BAUD as u64;
        match read_res {
            Ok(None) => info!(
                "round {}: stream OK, {} bytes in {} us (wire {} us)",
                round, STREAM_LEN, elapsed, wire
            ),
            Ok(Some(at)) => {
                failures += 1;
                error!("round {}: stream MISMATCH at {}", round, at);
            }
            Err(e) => {
                failures += 1;
                error!("round {}: stream read failed: {}", round, e);
            }
        }

        // --- Odd sizes ------------------------------------------------------------------
        let mut odd_ok = true;
        for size in 1..=13usize {
            let mut chunk = [0u8; 13];
            for (i, b) in chunk[..size].iter_mut().enumerate() {
                *b = pattern(i, round);
            }
            unwrap!(tx.write_all(&chunk[..size]).await);
            tx.flush().await;
            match read_and_check(&mut rx, size, round).await {
                Ok(None) => {}
                Ok(Some(at)) => {
                    odd_ok = false;
                    error!("round {}: {}-byte write MISMATCH at {}", round, size, at);
                }
                Err(e) => {
                    odd_ok = false;
                    error!("round {}: {}-byte write read failed: {}", round, size, e);
                }
            }
        }
        if odd_ok {
            info!("round {}: odd sizes OK", round);
        } else {
            failures += 1;
        }

        // --- Flush ----------------------------------------------------------------------
        // Once `flush` returns the last stop bit is on the wire, so the receiver must hold
        // every byte after a moment for its own DMA to land the last one.
        let mut burst = [0u8; 40];
        for (i, b) in burst.iter_mut().enumerate() {
            *b = pattern(i, round);
        }
        unwrap!(tx.write_all(&burst).await);
        tx.flush().await;
        Timer::after_micros(30).await;
        let have = rx.len();
        if have == burst.len() && tx.pending() == 0 {
            info!("round {}: flush OK", round);
        } else {
            failures += 1;
            error!(
                "round {}: flush returned with {} of {} bytes received, {} still pending",
                round,
                have,
                burst.len(),
                tx.pending()
            );
        }
        let mut scratch = [0u8; RX_POOL_LEN];
        while !rx.is_empty() {
            let _ = rx.blocking_read(&mut scratch);
        }

        if failures > 0 {
            warn!("{} failures across {} rounds", failures, round);
        }

        Timer::after_secs(1).await;
    }
}
