//! This example shows how to use the full-duplex PIO ring-buffered DMA UART on the RP235x.
//!
//! [`PioRingBufferedUart`] is a ring-buffered PIO receiver (with idle-line wakeup) and a
//! ring-buffered PIO transmitter, configured together. It is used here as a loopback: whatever
//! is sent is received back by the same driver.
//!
//! Each round checks every byte twice: first through the combined type, with a burst that fits
//! the receive pool, and then through the halves returned by `split_ref`, streaming several pool
//! sizes while reading concurrently.
//!
//! **Wiring: connect PIN_0 (PIO TX) to PIN_1 (PIO RX) with a jumper wire.**

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::dma_ring::State;
use embassy_rp::dma_ring_tx::TxInterruptHandler;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH2, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::uart::PioRingBufferedUart;
use embassy_rp::uart::Error;
use embassy_rp::{bind_interrupts, dma};
use embassy_time::Timer;
use embedded_io_async::{Read, Write};
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, TxInterruptHandler<DMA_CH2>;
});

const TX_POOL_LEN: usize = 64;
const RX_POOL_LEN: usize = 256;
const STREAM_LEN: usize = 1500;
/// Fits in the receive pool with room to spare, since nothing reads while it is written.
const COMBINED_LEN: usize = RX_POOL_LEN / 2;

/// Content of the stream at byte `i`.
fn pattern(i: usize, round: u32) -> u8 {
    (i as u8).wrapping_mul(31).wrapping_add(round as u8)
}

/// Read `len` bytes and return the offset of the first mismatch, if any.
async fn read_and_check(rx: &mut impl Read<Error = Error>, len: usize, round: u32) -> Result<Option<usize>, Error> {
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

    const BAUD: u32 = 1_000_000;

    static TX_POOL: StaticCell<[u8; TX_POOL_LEN]> = StaticCell::new();
    static RX_POOL: StaticCell<[u8; RX_POOL_LEN]> = StaticCell::new();
    // The DMA reads this table directly while the ring runs, so it has to outlive the driver.
    static RX_STATE: State = State::new();
    let tx_pool = &mut TX_POOL.init([0; TX_POOL_LEN])[..];
    let rx_pool = &mut RX_POOL.init([0; RX_POOL_LEN])[..];

    // TX on SM0, RX on SM1, idle detector on SM2. `irq2` must match the idle SM index.
    let Pio {
        mut common,
        sm0,
        sm1,
        sm2,
        irq2,
        irq_flags,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let mut uart = PioRingBufferedUart::new(
        BAUD,
        &mut common,
        sm0,
        sm1,
        sm2,
        irq2,
        irq_flags,
        p.PIN_0,
        p.PIN_1,
        p.DMA_CH0, // rx data channel
        p.DMA_CH1, // rx control channel
        p.DMA_CH2, // tx channel
        Irqs,
        &RX_STATE,
        rx_pool,
        tx_pool,
    );

    info!("full-duplex PIO ring-buffered UART loopback @ {} baud", BAUD);
    info!("jumper PIN_0 (PIO TX) -> PIN_1 (PIO RX)");

    let mut round: u32 = 0;
    let mut failures: u32 = 0;

    loop {
        round += 1;

        let mut data = [0u8; STREAM_LEN];
        for (i, b) in data.iter_mut().enumerate() {
            *b = pattern(i, round);
        }

        // Through the combined type's embedded-io-async impls. Written and read back
        // sequentially, so it has to fit in the receive pool.
        unwrap!(uart.write_all(&data[..COMBINED_LEN]).await);
        unwrap!(Write::flush(&mut uart).await);
        match read_and_check(&mut uart, COMBINED_LEN, round).await {
            Ok(None) => info!("round {}: combined type OK, {} bytes", round, COMBINED_LEN),
            Ok(Some(at)) => {
                failures += 1;
                error!("round {}: combined MISMATCH at {}", round, at);
            }
            Err(e) => {
                failures += 1;
                error!("round {}: combined read failed: {}", round, e);
            }
        }

        // Through the split halves, writing and reading concurrently.
        {
            let (tx, rx) = uart.split_ref();
            let (w, r) = join(
                async {
                    let r = tx.write_all(&data).await;
                    tx.flush().await;
                    r
                },
                read_and_check(rx, STREAM_LEN, round),
            )
            .await;
            unwrap!(w);
            match r {
                Ok(None) => info!("round {}: split halves OK, {} bytes", round, STREAM_LEN),
                Ok(Some(at)) => {
                    failures += 1;
                    error!("round {}: split MISMATCH at {}", round, at);
                }
                Err(e) => {
                    failures += 1;
                    error!("round {}: split read failed: {}", round, e);
                }
            }
        }

        if failures > 0 {
            warn!("{} failures across {} rounds", failures, round);
        }

        Timer::after_secs(1).await;
    }
}
