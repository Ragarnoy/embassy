//! Stress test for the ring-buffered DMA UART TX driver under heavy interrupt load.
//!
//! The transmit ring restarts its DMA channel from the channel's completion interrupt. If that
//! interrupt is held off, the channel sits idle with data queued; if the writer runs while it is
//! held off, the two must not both start a run or lose track of the queue. This test provokes
//! exactly that.
//!
//! A higher-priority interrupt executor (above the DMA interrupt) runs a task that alternates
//! random busy spins of up to ~1.2 ms with short sleeps, so the DMA interrupt is routinely
//! delayed for longer than it takes to send the whole 64-byte TX pool (640 us at 1 Mbaud). Each
//! round then streams a large buffer through the ring, with a deliberately small TX pool, and
//! checks every byte on a ring-buffered receiver.
//!
//! The receive pool is large so the receiver itself survives the starvation: the point is to
//! find faults in the *transmit* path.
//!
//! **Wiring: connect PIN_4 (UART1 TX) to PIN_1 (UART0 RX) with a jumper wire.**

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::dma_ring_tx::TxInterruptHandler;
use embassy_rp::executor::InterruptExecutor;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH2, UART0};
use embassy_rp::uart::{Config, Error, InterruptHandler, RingBufferedUartRx, RingBufferedUartTx};
use embassy_rp::{bind_interrupts, dma, interrupt};
use embassy_time::{Instant, Timer};
use embedded_io_async::Write;
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandler<UART0>;
    // The TX channel needs its own handler; it cannot share `dma::InterruptHandler`.
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, TxInterruptHandler<DMA_CH2>;
});

const BAUD: u32 = 1_000_000;

/// Deliberately small, so the stream wraps it and waits for space many times.
const TX_POOL_LEN: usize = 64;
const RX_POOL_LEN: usize = 4096;

/// Several pools' worth, so the writer has to wait on the DMA.
const STREAM_LEN: usize = 12_000;

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI_IRQ_1() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

/// Longest spin, in microseconds. Past the 640 us it takes to send the whole TX pool.
const MAX_SPIN_US: u32 = 1200;

/// Burn CPU at a priority above the DMA interrupt, at random.
#[embassy_executor::task]
async fn hog() {
    let mut x: u32 = 0x1234_5678;
    let mut longest = 0u32;
    let mut count = 0u32;
    loop {
        // xorshift32
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;

        let spin = 50 + x % MAX_SPIN_US;
        let until = Instant::now() + embassy_time::Duration::from_micros(spin as u64);
        while Instant::now() < until {}
        longest = longest.max(spin);
        count += 1;
        if count % 2000 == 0 {
            info!("hog: {} spins, longest {} us", count, longest);
        }

        Timer::after_micros(20 + (x >> 16) as u64 % 200).await;
    }
}

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
    let mut buf = [0u8; 256];
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

    interrupt::SWI_IRQ_1.set_priority(Priority::P2);
    let hog_spawner = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_1);
    hog_spawner.spawn(unwrap!(hog()));

    let mut config = Config::default();
    config.baudrate = BAUD;

    static TX_POOL: StaticCell<[u8; TX_POOL_LEN]> = StaticCell::new();
    let tx_pool = &mut TX_POOL.init([0; TX_POOL_LEN])[..];
    let mut tx = RingBufferedUartTx::new(p.UART1, p.PIN_4, p.DMA_CH2, Irqs, tx_pool, config);

    static RX_POOL: StaticCell<[u8; RX_POOL_LEN]> = StaticCell::new();
    let rx_pool = &mut RX_POOL.init([0; RX_POOL_LEN])[..];
    let mut rx = RingBufferedUartRx::new(p.UART0, p.PIN_1, p.DMA_CH0, p.DMA_CH1, Irqs, rx_pool, config);

    info!("ring-buffered UART TX stress test @ {} baud", BAUD);
    info!("jumper PIN_4 (UART1 TX) -> PIN_1 (UART0 RX)");
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
