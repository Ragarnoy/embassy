//! Pio backed uart drivers

use core::convert::Infallible;
use core::future::poll_fn;
use core::task::Poll;

use embassy_futures::select::select;
use embedded_io_async::{ErrorType, Read, Write};

use crate::Peri;
use crate::dma::ChannelInstance;
use crate::dma_ring::{RxRing, State};
use crate::gpio::Level;
use crate::interrupt::typelevel::Binding;
use crate::pio::{
    Common, Config, Direction as PioDirection, FifoJoin, Instance, Irq, IrqFlags, LoadedProgram, PioPin,
    ShiftDirection, StateMachine,
};
use crate::pio_programs::clock_divider::calculate_pio_clock_divider;
use crate::uart::Error;

///This struct is a unification of the PioRx and PioTx state machines.
pub struct PioUart<'d, P: Instance, const TX_SM: usize, const RX_SM: usize> {
    ///Transimiter half of the Pio Uart
    pub tx: PioUartTx<'d, P, TX_SM>,
    ///Receiver half of the Pio Uart
    pub rx: PioUartRx<'d, P, RX_SM>,
}

impl<'d, P, const TX_SM: usize, const RX_SM: usize> PioUart<'d, P, TX_SM, RX_SM>
where
    P: Instance,
{
    /// Configures a new instance of pio uart
    pub fn new(
        baud: u32,
        common: &mut Common<'d, P>,
        tx_sm: StateMachine<'d, P, TX_SM>,
        rx_sm: StateMachine<'d, P, RX_SM>,
        tx_pin: Peri<'d, impl PioPin>,
        rx_pin: Peri<'d, impl PioPin>,
    ) -> Self {
        let tx_prg = PioUartTxProgram::new(common);
        let rx_prg = PioUartRxProgram::new(common);
        Self {
            tx: PioUartTx::new(baud, common, tx_sm, tx_pin, &tx_prg),
            rx: PioUartRx::new(baud, common, rx_sm, rx_pin, &rx_prg),
        }
    }
    /// Split the Uart into a transmitter and receiver, which is particularly
    /// useful when having two tasks correlating to transmitting and receiving.
    pub fn split(self) -> (PioUartTx<'d, P, TX_SM>, PioUartRx<'d, P, RX_SM>) {
        (self.tx, self.rx)
    }
    /// Split the Uart into a transmitter and receiver by mutable reference,
    /// which is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split_ref(&mut self) -> (&mut PioUartTx<'d, P, TX_SM>, &mut PioUartRx<'d, P, RX_SM>) {
        (&mut self.tx, &mut self.rx)
    }
}

/// This struct represents a uart tx program loaded into pio instruction memory.
pub struct PioUartTxProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioUartTxProgram<'d, PIO> {
    /// Load the uart tx program into the given pio
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = pio::pio_asm!(
            r#"
                .side_set 1 opt

                ; An 8n1 UART transmit program.
                ; OUT pin 0 and side-set pin 0 are both mapped to UART TX pin.

                    nop        side 1 [7]  ; Stop bit/idle time
                    pull       side 1 [7]  ; Assert stop bit, or stall with line in idle state
                    set x, 7   side 0 [7]  ; Preload bit counter, assert start bit for 8 clocks
                bitloop:                   ; This loop will run 8 times (8n1 UART)
                    out pins, 1            ; Shift 1 bit from OSR to the first OUT pin
                    jmp x-- bitloop   [6]  ; Each loop iteration is 8 cycles.
            "#
        );

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

/// PIO backed Uart transmitter
pub struct PioUartTx<'d, PIO: Instance, const SM: usize> {
    sm_tx: StateMachine<'d, PIO, SM>,
}

impl<'d, PIO: Instance, const SM: usize> PioUartTx<'d, PIO, SM> {
    /// Configure a pio state machine to use the loaded tx program.
    pub fn new(
        baud: u32,
        common: &mut Common<'d, PIO>,
        mut sm_tx: StateMachine<'d, PIO, SM>,
        tx_pin: Peri<'d, impl PioPin>,
        program: &PioUartTxProgram<'d, PIO>,
    ) -> Self {
        let tx_pin = common.make_pio_pin(tx_pin);
        sm_tx.set_pins(Level::High, &[&tx_pin]);
        sm_tx.set_pin_dirs(PioDirection::Out, &[&tx_pin]);

        let mut cfg = Config::default();

        cfg.set_out_pins(&[&tx_pin]);
        cfg.use_program(&program.prg, &[&tx_pin]);
        cfg.shift_out.auto_fill = false;
        cfg.shift_out.direction = ShiftDirection::Right;
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.clock_divider = calculate_pio_clock_divider(8 * baud);
        sm_tx.set_config(&cfg);
        sm_tx.set_enable(true);

        Self { sm_tx }
    }

    /// Write a single u8
    pub async fn write_u8(&mut self, data: u8) {
        self.sm_tx.tx().wait_push(data as u32).await;
    }

    /// Write all bytes in `buf`.
    ///
    /// Returns once all bytes have been pushed into the state machine's TX FIFO.
    pub async fn write(&mut self, buf: &[u8]) {
        for byte in buf {
            self.write_u8(*byte).await;
        }
    }

    /// Wait until all written bytes have been fully transmitted on the wire.
    pub async fn flush(&mut self) {
        // There's no PIO interrupt for "TX FIFO empty" or "stalled", so poll.
        while !self.sm_tx.tx().empty() {
            embassy_futures::yield_now().await;
        }
        // The FIFO is empty but the SM may still be shifting out the last byte. It stalls
        // on `pull` once the stop bit is done, so clear the stall flag and wait for it to be set again.
        let _ = self.sm_tx.tx().stalled();
        while !self.sm_tx.tx().stalled() {
            embassy_futures::yield_now().await;
        }
    }

    /// Change baud rate on run time  
    pub fn set_baudrate(&mut self, baud: u32) {
        let clock_divider = calculate_pio_clock_divider(8 * baud);
        self.sm_tx.set_enable(false);
        self.sm_tx.clear_fifos();
        self.sm_tx.restart();
        self.sm_tx.set_clock_divider(clock_divider);
        self.sm_tx.set_enable(true);
    }
}

impl<PIO: Instance, const SM: usize> ErrorType for PioUartTx<'_, PIO, SM> {
    type Error = Infallible;
}

impl<PIO: Instance, const SM: usize> Write for PioUartTx<'_, PIO, SM> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Infallible> {
        PioUartTx::write(self, buf).await;
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Infallible> {
        PioUartTx::flush(self).await;
        Ok(())
    }
}

/// This struct represents a Uart Rx program loaded into pio instruction memory.
pub struct PioUartRxProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioUartRxProgram<'d, PIO> {
    /// Load the uart rx program into the given pio
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = pio::pio_asm!(
            r#"
                ; Slightly more fleshed-out 8n1 UART receiver which handles framing errors and
                ; break conditions more gracefully.
                ; IN pin 0 and JMP pin are both mapped to the GPIO used as UART RX.

                start:
                    wait 0 pin 0        ; Stall until start bit is asserted
                    set x, 7    [10]    ; Preload bit counter, then delay until halfway through
                rx_bitloop:             ; the first data bit (12 cycles incl wait, set).
                    in pins, 1          ; Shift data bit into ISR
                    jmp x-- rx_bitloop [6] ; Loop 8 times, each loop iteration is 8 cycles
                    jmp pin good_rx_stop  ; Check stop bit (should be high)

                    irq 4 rel           ; Either a framing error or a break. Set a sticky flag,
                    wait 1 pin 0        ; and wait for line to return to idle state.
                    jmp start           ; Don't push data if we didn't see good framing.

                good_rx_stop:           ; No delay before returning to start; a little slack is
                    in null 24
                    push                ; important in case the TX clock is slightly too fast.
            "#
        );

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

/// PIO backed Uart receiver
pub struct PioUartRx<'d, PIO: Instance, const SM: usize> {
    sm_rx: StateMachine<'d, PIO, SM>,
}

impl<'d, PIO: Instance, const SM: usize> PioUartRx<'d, PIO, SM> {
    /// Configure a pio state machine to use the loaded rx program.
    pub fn new(
        baud: u32,
        common: &mut Common<'d, PIO>,
        mut sm_rx: StateMachine<'d, PIO, SM>,
        rx_pin: Peri<'d, impl PioPin>,
        program: &PioUartRxProgram<'d, PIO>,
    ) -> Self {
        let mut cfg = Config::default();
        cfg.use_program(&program.prg, &[]);

        let mut rx_pin = common.make_pio_pin(rx_pin);
        rx_pin.set_pull(crate::gpio::Pull::Up);
        cfg.set_in_pins(&[&rx_pin]);
        cfg.set_jmp_pin(&rx_pin);
        sm_rx.set_pins(Level::High, &[&rx_pin]);

        cfg.clock_divider = calculate_pio_clock_divider(8 * baud);
        cfg.shift_in.auto_fill = false;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.shift_in.threshold = 32;
        cfg.fifo_join = FifoJoin::RxOnly;
        sm_rx.set_pin_dirs(PioDirection::In, &[&rx_pin]);
        sm_rx.set_config(&cfg);
        sm_rx.set_enable(true);

        Self { sm_rx }
    }

    /// Wait for a single u8
    pub async fn read_u8(&mut self) -> u8 {
        self.sm_rx.rx().wait_pull().await as u8
    }

    /// Read bytes until `buf` is full.
    pub async fn read(&mut self, buf: &mut [u8]) {
        for byte in buf {
            *byte = self.read_u8().await;
        }
    }

    /// Change Baud rate on runtime
    pub fn set_baudrate(&mut self, baud: u32) {
        let clock_divider = calculate_pio_clock_divider(8 * baud);
        self.sm_rx.set_enable(false);
        self.sm_rx.clear_fifos();
        self.sm_rx.restart();
        self.sm_rx.set_clock_divider(clock_divider);
        self.sm_rx.set_enable(true);
    }
}

impl<PIO: Instance, const SM: usize> ErrorType for PioUartRx<'_, PIO, SM> {
    type Error = Infallible;
}

impl<PIO: Instance, const SM: usize> Read for PioUartRx<'_, PIO, SM> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Infallible> {
        PioUartRx::read(self, buf).await;
        Ok(buf.len())
    }
}

/// Program that raises a PIO interrupt once the RX line has gone idle.
///
/// This runs on its own state machine watching the same pin as [`PioUartRxProgram`], which it
/// leaves completely untouched. It exists because the PL011 cannot detect an idle line behind
/// DMA at all: with `RXDMAE` set the RX DREQ asserts on any non-empty FIFO, so the DMA drains
/// it long before the receive-timeout condition can hold.
///
/// # Threshold
///
/// The countdown is `set y, 20` with a `[4]` delay on the loop instruction. One iteration is
/// `jmp pin` (1 cycle) plus `jmp y--` (1 + 4), so the threshold is `21 * 6 = 126` cycles. The
/// state machine runs at `8 * baud`, making that **15.75 bit times**, independent of baud.
///
/// The floor is 9 bit times: back-to-back `0xff` bytes hold the line high for 8 data bits plus
/// the stop bit before the next start bit, so a shorter threshold would fire in the middle of a
/// stream. Firing early is harmless — the reader wakes, takes what is there and returns — so the
/// margin above 9 is for clock tolerance rather than correctness.
///
/// These numbers live in the assembly below, which is authoritative; `pio_asm!` takes a literal
/// so they cannot be pulled from Rust constants. Both have been reasoned about but neither has
/// been checked against real traffic yet.
pub struct PioUartIdleProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioUartIdleProgram<'d, PIO> {
    /// Load the program into the given PIO instance.
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = pio::pio_asm!(
            r#"
                ; Idle-line detector. IN pin 0 and JMP pin are both the UART RX GPIO.
                ; Raises `irq 0 rel`, i.e. flag (0 + SM), once the line has been continuously
                ; high for the countdown. Only flags 0..3 can reach the host, and `rel` keeps
                ; the flag index equal to this state machine's index, which is always < 4.

                idle_wait:
                    wait 0 pin 0        ; Stall until the line goes low: a start bit.
                    wait 1 pin 0        ; Then until it comes back up.
                    set y, 20           ; see "Threshold" above
                count:
                    jmp pin still_idle  ; Still high?
                    jmp idle_wait       ; No: a new frame began, start over without firing.
                still_idle:
                    jmp y-- count [4]
                    irq 0 rel           ; Countdown expired: the line is idle.
                    jmp idle_wait
            "#
        );

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

/// PIO backed UART receiver with a continuous DMA ring and idle-line wakeup.
///
/// Like [`RingBufferedUartRx`](crate::uart::RingBufferedUartRx) this receives continuously into
/// a pool of DMA buffers with no per-byte interrupt, so it keeps up at baud rates where a
/// byte-at-a-time receiver drops data. See [`crate::dma_ring`] for the DMA construction.
///
/// What it adds over the hardware-UART driver is **idle-line wakeup**: a second state machine
/// watches the line and raises an interrupt once it goes quiet, so a blocked [`read`](Self::read)
/// returns at the end of a frame instead of waiting for a DMA segment to fill. That is the one
/// thing the PL011 cannot do behind DMA.
///
/// Costs two state machines, two DMA channels and 18 of the 32 PIO instruction slots.
pub struct PioRingBufferedUartRx<'d, PIO: Instance, const SM: usize, const SM_IDLE: usize> {
    ring: RxRing<'d>,
    /// Held so the state machines stay claimed and are disabled on drop.
    _sm_rx: StateMachine<'d, PIO, SM>,
    _sm_idle: StateMachine<'d, PIO, SM_IDLE>,
    idle_irq: Irq<'d, PIO, SM_IDLE>,
    irq_flags: IrqFlags<'d, PIO>,
}

impl<'d, PIO: Instance, const SM: usize, const SM_IDLE: usize> PioRingBufferedUartRx<'d, PIO, SM, SM_IDLE> {
    /// Configure two state machines and a DMA ring for continuous reception.
    ///
    /// `rx_buffer` is the DMA pool; its length must be a non-zero multiple of 4, and it is split
    /// into 4 segments internally. `state` holds the DMA-visible pointer table and must outlive
    /// the driver — a plain `static` works, as [`State`] is `Sync`.
    ///
    /// `idle_irq` must be the [`Irq`] whose index equals `SM_IDLE`, because the idle program
    /// raises its flag with `irq 0 rel`. Only `rx_dma` needs an interrupt binding; the control
    /// channel runs silently.
    #[allow(clippy::too_many_arguments)]
    pub fn new<RxDma: ChannelInstance, CtrlDma: ChannelInstance>(
        baud: u32,
        common: &mut Common<'d, PIO>,
        mut sm_rx: StateMachine<'d, PIO, SM>,
        mut sm_idle: StateMachine<'d, PIO, SM_IDLE>,
        idle_irq: Irq<'d, PIO, SM_IDLE>,
        irq_flags: IrqFlags<'d, PIO>,
        rx_pin: Peri<'d, impl PioPin>,
        rx_program: &PioUartRxProgram<'d, PIO>,
        idle_program: &PioUartIdleProgram<'d, PIO>,
        rx_dma: Peri<'d, RxDma>,
        ctrl_dma: Peri<'d, CtrlDma>,
        irq: impl Binding<RxDma::Interrupt, crate::dma::InterruptHandler<RxDma>> + 'd,
        state: &'d State,
        rx_buffer: &'d mut [u8],
    ) -> Self {
        let clock_divider = calculate_pio_clock_divider(8 * baud);

        let mut rx_pin = common.make_pio_pin(rx_pin);
        rx_pin.set_pull(crate::gpio::Pull::Up);

        // Receiver: exactly the configuration `PioUartRx::new` uses. The program pushes a
        // normalised 32-bit word (`in null 24` before `push`), so the byte lands in bits 7:0
        // and a byte-wide DMA read straight off RXF picks it up.
        let mut cfg = Config::default();
        cfg.use_program(&rx_program.prg, &[]);
        cfg.set_in_pins(&[&rx_pin]);
        cfg.set_jmp_pin(&rx_pin);
        cfg.clock_divider = clock_divider;
        cfg.shift_in.auto_fill = false;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.shift_in.threshold = 32;
        cfg.fifo_join = FifoJoin::RxOnly;
        sm_rx.set_pins(Level::High, &[&rx_pin]);
        sm_rx.set_pin_dirs(PioDirection::In, &[&rx_pin]);
        sm_rx.set_config(&cfg);

        // Idle detector: same pin, same clock, no shifting.
        let mut idle_cfg = Config::default();
        idle_cfg.use_program(&idle_program.prg, &[]);
        idle_cfg.set_in_pins(&[&rx_pin]);
        idle_cfg.set_jmp_pin(&rx_pin);
        idle_cfg.clock_divider = clock_divider;
        sm_idle.set_pin_dirs(PioDirection::In, &[&rx_pin]);
        sm_idle.set_config(&idle_cfg);

        // Clear state a previous run may have left behind: a latched framing or idle flag would
        // otherwise be reported against the first byte of this one, and a byte still sitting in
        // the RX FIFO would be picked up by the DMA as if it had just arrived.
        irq_flags.clear(Self::ERR_FLAG as usize);
        irq_flags.clear(SM_IDLE);
        sm_rx.clear_fifos();

        let data = crate::dma::Channel::new(rx_dma, irq);
        // The control channel never raises an interrupt, so it does not need a binding.
        let control = crate::dma::Channel::new_no_interrupt(ctrl_dma);

        let ring = RxRing::new(
            state,
            sm_rx.rx_fifo_ptr() as *const u8,
            sm_rx.rx_treq(),
            data,
            control,
            rx_buffer,
        );

        // Start receiving only once the ring is draining the FIFO, so a byte arriving during
        // setup cannot sit in the FIFO unaccounted for.
        sm_rx.set_enable(true);
        sm_idle.set_enable(true);

        Self {
            ring,
            _sm_rx: sm_rx,
            _sm_idle: sm_idle,
            idle_irq,
            irq_flags,
        }
    }

    /// PIO interrupt flag the receive program raises on a framing error or break.
    ///
    /// `irq 4 rel` in [`PioUartRxProgram`], i.e. `4 + SM`. Flags 4..7 cannot reach the host, so
    /// this is polled rather than awaited — which is fine, since the reader is already awake
    /// whenever it checks.
    const ERR_FLAG: u8 = 4 + SM as u8;

    /// Take and clear a latched framing error.
    ///
    /// The receive program uses one flag for both framing errors and break conditions, so both
    /// surface as [`Error::Framing`].
    fn take_error(&mut self) -> Option<Error> {
        if self.irq_flags.check(Self::ERR_FLAG) {
            self.irq_flags.clear(Self::ERR_FLAG as usize);
            Some(Error::Framing)
        } else {
            None
        }
    }

    /// Read bytes into `buf`, waiting until at least one is available.
    ///
    /// Returns as soon as either the line goes idle or the DMA finishes a segment, whichever
    /// comes first. Returns the number of bytes read, which is never zero unless `buf` is empty.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        loop {
            if let Some(e) = self.take_error() {
                return Err(e);
            }
            match self.ring.try_read(buf) {
                Ok(0) => {}
                Ok(n) => return Ok(n),
                Err(()) => return Err(Error::Overrun),
            }

            // Park until the DMA finishes a segment or the line goes idle. The inner poll_fn
            // registers before re-checking, so data landing between the try_read above and
            // parking is not lost. The idle flag is sticky and `Irq::wait` checks it before
            // registering, so recreating the future each pass cannot miss a fire either.
            let ring = &mut self.ring;
            select(
                poll_fn(|cx| {
                    ring.register_waker(cx.waker());
                    if ring.is_empty() {
                        Poll::Pending
                    } else {
                        Poll::Ready(())
                    }
                }),
                self.idle_irq.wait(),
            )
            .await;
        }
    }

    /// Read whatever is already buffered, without waiting.
    ///
    /// Returns `Ok(0)` if nothing has arrived yet.
    pub fn blocking_read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        if let Some(e) = self.take_error() {
            return Err(e);
        }
        self.ring.try_read(buf).map_err(|()| Error::Overrun)
    }

    /// Number of bytes currently available to read.
    pub fn len(&mut self) -> usize {
        self.ring.len()
    }

    /// Whether any data is currently available to read.
    pub fn is_empty(&mut self) -> bool {
        self.ring.is_empty()
    }

    /// Whether a call to [`read`](Self::read) would return without waiting.
    pub fn read_ready(&mut self) -> Result<bool, Error> {
        Ok(!self.is_empty())
    }
}

impl<'d, PIO: Instance, const SM: usize, const SM_IDLE: usize> ErrorType
    for PioRingBufferedUartRx<'d, PIO, SM, SM_IDLE>
{
    type Error = Error;
}

impl<'d, PIO: Instance, const SM: usize, const SM_IDLE: usize> Read for PioRingBufferedUartRx<'d, PIO, SM, SM_IDLE> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        Self::read(self, buf).await
    }
}

impl<'d, PIO: Instance, const SM: usize, const SM_IDLE: usize> embedded_io_async::ReadReady
    for PioRingBufferedUartRx<'d, PIO, SM, SM_IDLE>
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Self::read_ready(self)
    }
}
