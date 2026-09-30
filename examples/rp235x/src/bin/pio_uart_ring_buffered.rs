//! Self-test for the PIO-backed ring-buffered DMA UART RX driver on the RP235x.
//!
//! [`PioRingBufferedUartRx`] receives continuously into a pool of DMA buffers driven by two
//! chained DMA channels, exactly like the hardware-UART [`RingBufferedUartRx`]. What it adds is
//! **idle-line wakeup**: a second PIO state machine watches the line and raises an interrupt once
//! it goes quiet, so a blocked read returns at the end of a frame rather than waiting for a DMA
//! segment to fill.
//!
//! That is the one thing the PL011 cannot do behind DMA — with `RXDMAE` set its RX DREQ asserts
//! on any non-empty FIFO, so the DMA drains it long before the receive-timeout condition can
//! hold and the interrupt never fires.
//!
//! Three phases:
//!
//! - **Capacity**: send a burst that fits in the pool while the receiving task is parked in the
//!   TX await, then read it back and verify every byte.
//! - **Idle latency**: send a frame far smaller than one segment, concurrently with a pending
//!   read, and measure how long the read takes to return. This is the phase that justifies the
//!   driver: it should return shortly after the frame ends, not after a whole segment fills.
//!   The equivalent figure for the PL011 driver is printed alongside for comparison.
//! - **Break**: assert a break condition and confirm it surfaces as an error rather than as
//!   silent corruption. The PIO receive program raises one flag for both framing errors and
//!   breaks, so both report [`Error::Framing`].
//!
//! **Wiring: connect PIN_4 (UART1 TX) to PIN_5 (PIO RX) with a jumper wire.**

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::dma_ring::State;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH2, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::uart::{PioRingBufferedUartRx, PioUartIdleProgram, PioUartRxProgram};
use embassy_rp::uart::{Config, Error, UartTx};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Instant, Timer};
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH2>;
});

const BAUD: u32 = 1_000_000;

/// The RX DMA pool, split into 4 segments internally.
const RX_POOL_LEN: usize = 256;
const SEG_LEN: usize = RX_POOL_LEN / 4;

/// Bytes sent in the capacity phase. Usable capacity is the pool minus one segment, so stay
/// comfortably under that rather than exactly on the boundary.
const BURST_LEN: usize = RX_POOL_LEN / 2;

/// Bytes sent in the idle-latency phase — deliberately far smaller than one segment, so that
/// waiting for a segment boundary would be plainly visible in the timing.
const SHORT_FRAME_LEN: usize = 5;

/// Content of the stream at byte `i`. Any dropped or duplicated byte shifts the sequence and
/// shows up as a mismatch.
fn pattern(i: usize, round: u32) -> u8 {
    (i as u8).wrapping_mul(31).wrapping_add(round as u8)
}

/// Microseconds one byte spends on the wire at `BAUD`, 8N1: start + 8 data + stop.
const fn byte_micros() -> u64 {
    10 * 1_000_000 / BAUD as u64
}

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut config = Config::default();
    config.baudrate = BAUD;

    let mut tx = UartTx::new(p.UART1, p.PIN_4, p.DMA_CH2, Irqs, config);

    let Pio {
        mut common,
        sm0,
        sm1,
        irq1,
        irq_flags,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let rx_program = PioUartRxProgram::new(&mut common);
    let idle_program = PioUartIdleProgram::new(&mut common);

    // The DMA reads this table directly while the ring runs, so it has to outlive the driver.
    // `State` is `Sync`, so a plain static is enough.
    static RX_STATE: State = State::new();

    static RX_POOL: StaticCell<[u8; RX_POOL_LEN]> = StaticCell::new();
    let rx_pool = &mut RX_POOL.init([0; RX_POOL_LEN])[..];

    // RX on SM0, idle detector on SM1. `irq1` must match the idle SM index: the idle program
    // raises its flag with `irq 0 rel`, so the flag number is the state machine's own index.
    let mut rx = PioRingBufferedUartRx::new(
        BAUD,
        &mut common,
        sm0,
        sm1,
        irq1,
        irq_flags,
        p.PIN_5,
        &rx_program,
        &idle_program,
        p.DMA_CH0, // data channel
        p.DMA_CH1, // control channel
        Irqs,
        &RX_STATE,
        rx_pool,
    );

    info!("PIO ring-buffered UART RX self-test @ {} baud", BAUD);
    info!("jumper PIN_4 (UART1 TX) -> PIN_5 (PIO RX)");
    info!("pool {} bytes, segment {} bytes", RX_POOL_LEN, SEG_LEN);
    info!(
        "one byte is ~{} us on the wire; a full segment is ~{} us",
        byte_micros(),
        byte_micros() * SEG_LEN as u64
    );

    let mut round: u32 = 0;
    let mut failures: u32 = 0;

    loop {
        round += 1;

        // --- Capacity -------------------------------------------------------------------
        // The whole burst arrives while this task is parked in the TX await, so nothing is
        // polling the receiver. The DMA ring has to capture all of it unaided.
        let mut sent = [0u8; BURST_LEN];
        for (i, b) in sent.iter_mut().enumerate() {
            *b = pattern(i, round);
        }
        unwrap!(tx.write(&sent).await);
        Timer::after_millis(5).await;

        let mut got = [0u8; BURST_LEN];
        let mut n = 0;
        let mut err = None;
        while n < BURST_LEN {
            match rx.read(&mut got[n..]).await {
                Ok(k) => n += k,
                Err(e) => {
                    err = Some(e);
                    break;
                }
            }
        }
        match err {
            None if got == sent => info!("round {}: capacity OK, {} bytes verified", round, BURST_LEN),
            None => {
                failures += 1;
                let first_bad = (0..BURST_LEN).find(|&i| got[i] != sent[i]);
                error!("round {}: MISMATCH at {:?}", round, first_bad);
            }
            Some(e) => {
                failures += 1;
                error!("round {}: capacity phase failed: {}", round, e);
            }
        }

        // --- Idle latency ---------------------------------------------------------------
        // Start the read *before* the frame goes out, so it is genuinely parked when the bytes
        // arrive. With idle detection it wakes about one character after the last byte; without
        // it, it would sit there until the DMA filled a whole segment.
        let mut frame = [0u8; SHORT_FRAME_LEN];
        for (i, b) in frame.iter_mut().enumerate() {
            *b = pattern(i, round);
        }
        let mut short_got = [0u8; SHORT_FRAME_LEN];

        let t0 = Instant::now();
        let (_, read_res) = join(
            async {
                unwrap!(tx.write(&frame).await);
            },
            rx.read(&mut short_got),
        )
        .await;
        let elapsed = t0.elapsed().as_micros();

        // What the wire alone costs, and what waiting for a segment would have cost.
        let wire = byte_micros() * SHORT_FRAME_LEN as u64;
        let segment_wait = byte_micros() * SEG_LEN as u64;

        match read_res {
            Ok(k) => {
                info!(
                    "round {}: idle wake returned {} byte(s) in {} us (wire {} us, segment wait would be {} us)",
                    round, k, elapsed, wire, segment_wait
                );
                if elapsed >= segment_wait {
                    failures += 1;
                    error!(
                        "round {}: took {} us, no better than waiting for a segment ({} us) -- idle detection is not working",
                        round, elapsed, segment_wait
                    );
                }
                if short_got[..k] != frame[..k] {
                    failures += 1;
                    error!("round {}: idle phase data mismatch", round);
                }
            }
            Err(e) => {
                failures += 1;
                error!("round {}: idle phase failed: {}", round, e);
            }
        }

        // Drain any remainder of the short frame so the next round starts clean.
        let mut scratch = [0u8; RX_POOL_LEN];
        while !rx.is_empty() {
            let _ = rx.blocking_read(&mut scratch);
        }

        // --- Break ----------------------------------------------------------------------
        // Every fifth round, assert a break and confirm it is reported. The receive program
        // flags framing errors and breaks through the same PIO IRQ, so both arrive as `Framing`.
        if round % 5 == 0 {
            tx.send_break(20).await;
            Timer::after_millis(5).await;

            let mut sink = [0u8; 16];
            match rx.read(&mut sink).await {
                Err(Error::Framing) => info!("round {}: break correctly reported", round),
                Err(e) => {
                    failures += 1;
                    error!("round {}: expected Framing, got {}", round, e);
                }
                Ok(k) => {
                    failures += 1;
                    error!("round {}: expected Framing, but read returned {} bytes", round, k);
                }
            }

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
