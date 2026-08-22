//! Timestamping marker-pin edges in PIO.
//!
//! The convention this implements: a case toggles a designated GPIO around the
//! events it cares about, and the fixture records when each edge arrived
//! against its own clock. One pin on the schematic, and it is what turns
//! "the driver did not complain" into a number — PWM frequency and duty, UART
//! baud, SPI clock rate, IRQ latency, DMA completion, page-flip interval.
//!
//! # Why PIO rather than an interrupt
//!
//! A GPIO interrupt on the RP2040 costs somewhere around 20-30 cycles of entry
//! before anything can read a timer, and that cost is *variable* — it depends
//! on what the core was doing. Measuring a Pi's IRQ latency with a bench whose
//! own latency is unknown and jittery answers nothing. A PIO state machine has
//! no such thing: it executes one instruction per cycle, always, with no
//! interrupts, no cache and no bus arbitration in its path.
//!
//! # The program
//!
//! X is a free-running down-counter. Both waiting loops are exactly two
//! instructions, so X ticks once per two system clocks whatever the pin is
//! doing, and `in x, 32` snapshots it into the FIFO the moment an edge lands:
//!
//! ```text
//! wait_high:
//!     jmp pin, got_high     ; pin high -> leave the loop
//!     jmp x--, wait_high    ; else tick and go round
//! got_high:
//!     in x, 32              ; autopush: timestamp of the rising edge
//! wait_low:
//!     jmp pin, low_again    ; pin still high -> keep waiting
//!     jmp got_low           ; pin low -> leave the loop
//! low_again:
//!     jmp x--, wait_low     ; tick and go round
//! got_low:
//!     in x, 32              ; timestamp of the falling edge
//! ```
//!
//! Two consequences worth knowing before trusting a number out of this:
//!
//! - **Resolution is two system clocks**, 16 ns at 125 MHz. Good enough to
//!   measure a single interval to about a percent at 1 µs, and irrelevant for
//!   a *rate*, which is measured across many periods and divides the error
//!   down with them.
//! - **The two edges are not detected with identical latency.** Leaving
//!   `wait_high` costs one instruction and leaving `wait_low` costs two, so a
//!   falling edge is stamped up to one cycle later than a rising one would
//!   have been. 8 ns, systematic, and it cancels entirely out of any
//!   measurement between two edges of the same direction — which is what a
//!   period is.
//!
//! # Depth
//!
//! The RX FIFO is four words, which at any interesting edge rate is a few
//! microseconds of headroom, so the FIFO is not the buffer — DMA drains it
//! into RAM and *that* is the buffer. The state machine stalls if it ever
//! fills, which is recorded rather than papered over: a capture that silently
//! dropped edges in the middle is worse than one that admits it, because the
//! intervals either side of the gap still look perfectly plausible.

use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::pac;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{
    Config, Direction, LoadedProgram, Pio, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::Peri;
/// The always-running 1 MHz timer block [`spin_us`] waits on.
///
/// `rp-pac` calls it `TIMER` on the RP2040 and `TIMER0` on the RP2350, which
/// has two. `embassy_rp`'s time driver aliases the same way and picks the same
/// block, which is what keeps this counter the one `embassy_time` is built on
/// rather than a second, independently started one.
#[cfg(feature = "rp2040")]
use pac::TIMER;
#[cfg(feature = "rp235x")]
use pac::TIMER0 as TIMER;

/// How many edges one capture holds.
///
/// 16 KB of the RP2040's 264 KB, which is not the constraint — the readout is.
/// At 15 timestamps per USB packet a full buffer is 273 round trips, a little
/// under a second, and a capture nobody can afford to read back is not depth.
pub const CAPACITY: usize = 4096;

/// DMA channel draining the state machine's RX FIFO.
///
/// Named by number rather than held as a `Peri`, because the capture is
/// deliberately fire-and-forget: `embassy_rp`'s `Transfer` aborts the channel
/// when it is dropped, so using it here would mean keeping a future alive
/// across the control requests that start and finish a capture. Nothing else
/// in this firmware uses DMA, and the pin allocation table in the README is
/// where a second user would have to declare itself.
const DMA_CHANNEL: usize = 0;

/// Where the DMA writes. Read back a packet at a time by `MARKER_READ`.
static mut SAMPLES: [u32; CAPACITY] = [0; CAPACITY];

/// The marker input, and the state machine watching it.
pub struct Marker<'d> {
    sm: StateMachine<'d, PIO0, 0>,
    /// Ticks per second of the timebase X counts in — reported rather than
    /// assumed by the host, since it follows the system clock and a firmware
    /// that changed the clock would otherwise silently rescale every
    /// measurement ever taken with it.
    tick_hz: u32,
    /// GPIO number of the marker pin, for the self-test's benefit.
    pin: u8,
    /// Where the program was loaded, so arming can put the program counter
    /// back on its first instruction.
    origin: u8,
}

impl<'d> Marker<'d> {
    /// Loads the program and starts the state machine watching `pin`.
    ///
    /// Left running from here on. The counter free-runs and the FIFO fills and
    /// stalls when nobody is capturing, which is harmless — [`Marker::arm`]
    /// resets the counter, the FIFO, the program counter and the stall latch
    /// alike — and it means an armed capture starts on the very next edge
    /// rather than after a start-up transient the host would have to know to
    /// discard.
    ///
    /// One property of a level-triggered wait loop worth stating: arming while
    /// the pin is already high stamps immediately, because `wait_high` has
    /// nothing to wait for. A case that wants its first timestamp to be a real
    /// edge drives the marker low before announcing itself.
    pub fn new(pio: Peri<'d, PIO0>, pin: Peri<'d, impl embassy_rp::pio::PioPin>) -> Self {
        let Pio {
            mut common,
            mut sm0,
            ..
        } = Pio::new(pio, super::Irqs);

        let program = embassy_rp::pio::program::pio_asm!(
            ".wrap_target",
            "wait_high:",
            "    jmp pin, got_high",
            "    jmp x--, wait_high",
            "got_high:",
            "    in x, 32",
            "wait_low:",
            "    jmp pin, low_again",
            "    jmp got_low",
            "low_again:",
            "    jmp x--, wait_low",
            "got_low:",
            "    in x, 32",
            ".wrap",
        );
        let loaded: LoadedProgram<'d, PIO0> = common.load_program(&program.program);

        let marker = common.make_pio_pin(pin);
        let pin_number = marker.pin();

        let mut config = Config::default();
        config.use_program(&loaded, &[]);
        config.set_jmp_pin(&marker);
        // Divider of exactly 1: the whole point is resolution, and every
        // fraction of a divider is a fraction of the answer thrown away.
        config.clock_divider = 1u8.into();
        config.shift_in = ShiftConfig {
            // Autopush at 32 bits, so `in x, 32` is one instruction and one
            // FIFO entry. An explicit `push` would cost a second cycle on
            // every edge and put it in the middle of the measurement.
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Left,
        };
        sm0.set_config(&config);
        // Input, explicitly. The pin is only ever watched: a marker the
        // fixture could drive would be a second driver on a net a case is
        // driving by definition, since the whole point is that the case
        // toggles it.
        sm0.set_pin_dirs(Direction::In, &[&marker]);

        let mut marker = Self {
            sm: sm0,
            // One tick is two system clocks: both waiting loops are two
            // instructions and PIO retires one per clock.
            tick_hz: clk_sys_freq() / 2,
            pin: pin_number,
            origin: loaded.origin,
        };
        marker.restart_counter();
        marker
    }

    /// Ticks per second of the capture timebase.
    pub fn tick_hz(&self) -> u32 {
        self.tick_hz
    }

    /// GPIO number the fixture watches for marker edges.
    pub fn pin(&self) -> u8 {
        self.pin
    }

    /// Resets the counter, the program counter and the FIFO, and points the
    /// DMA at the buffer.
    ///
    /// Everything is torn down before anything is started: an armed capture
    /// must not begin with edges that arrived while the host was still setting
    /// it up, because their timestamps are against the *previous* counter and
    /// would read as an enormous first interval.
    ///
    /// The stall latch is consumed here too, so [`Marker::overflowed`] answers
    /// for this capture and not for whatever happened before it. Power-cycling
    /// a board is enough to set it: the marker line is undriven for the
    /// duration, and a floating input generates edges faster than the DMA
    /// drains them, so the FIFO fills and the state machine stalls. Which is
    /// also why the jump in `restart_counter` matters — a stall is what parks
    /// the program counter mid-program in the first place.
    pub fn arm(&mut self) {
        self.stop_dma();
        self.restart_counter();
        let _ = self.sm.rx().stalled();
        self.start_dma();
    }

    /// How many edges the current capture has recorded.
    ///
    /// Derived from what the DMA has left to do rather than counted
    /// separately, so it cannot disagree with what is actually in the buffer.
    pub fn captured(&self) -> u16 {
        let remaining = trans_count(pac::DMA.ch(DMA_CHANNEL)) as usize;
        CAPACITY.saturating_sub(remaining).min(u16::MAX as usize) as u16
    }

    /// True if the state machine stalled trying to push a timestamp during
    /// this capture, i.e. edges arrived faster than the DMA drained them and
    /// some are missing from the middle of it.
    ///
    /// Scoped to the capture by [`Marker::arm`] consuming the latch, since the
    /// underlying flag is sticky and set by ordinary idling. Reading is itself
    /// destructive — the latch clears — so a caller that wants the answer
    /// twice has to keep it.
    pub fn overflowed(&mut self) -> bool {
        self.sm.rx().stalled()
    }

    /// Copies timestamps `start..start + count` out as little-endian `u32`s,
    /// returning how many bytes were written.
    ///
    /// The counter descends — `jmp x--` is the only one-cycle decrement PIO
    /// has — so the raw values fall as time passes. They are inverted here
    /// rather than at the far end: differences come out identical either way,
    /// but a timebase that runs backwards is the sort of thing a host-side
    /// analysis gets right the first time and wrong the third.
    pub fn read(&self, start: usize, count: usize, out: &mut [u8]) -> usize {
        let mut written = 0;
        for index in start..(start + count).min(CAPACITY) {
            // SAFETY: `SAMPLES` is written only by the DMA channel this module
            // owns, and read only here. A read racing the DMA sees either the
            // old or the new word, never a torn one — the transfers are
            // 32-bit and aligned — and `captured()` is what tells a caller
            // which words are meaningful.
            let raw =
                unsafe { core::ptr::read_volatile((&raw const SAMPLES).cast::<u32>().add(index)) };
            let ascending = !raw;
            out[written..written + 4].copy_from_slice(&ascending.to_le_bytes());
            written += 4;
        }
        written
    }

    /// Drives `count` pulses on the marker pin, for testing the bench itself.
    ///
    /// The fixture watching a signal it generated proves the capture path —
    /// PIO program, FIFO, DMA, readout — without a board, a wire or a case in
    /// the picture. That is worth a command of its own: when a board
    /// measurement comes out wrong, the first question is whose fault it is,
    /// and this answers it in one call.
    ///
    /// The pad is handed to SIO for the duration and given back afterwards.
    /// PIO reads GPIO inputs whatever the function select says, so the state
    /// machine goes on timestamping throughout.
    ///
    /// The half-period is timed against the RP2040's microsecond timer, not
    /// against a counted instruction loop. `cortex_m::asm::delay` was the
    /// obvious first choice and was wrong by exactly 3×: it iterates
    /// `1 + cycles / 2` times and each `subs`/`bne` costs about six cycles
    /// here rather than the two it assumes. The capture caught it — the
    /// fixture measured 600 µs periods where 200 µs had been asked for, and
    /// the host's own wall clock agreed with the fixture — which is a fair
    /// advertisement for why the bench measures rather than asserts. A
    /// hardware counter has no such calibration to get wrong.
    ///
    /// Busy-waits, which is why the caller's bounds matter: this blocks the
    /// control loop, and with it the USB stack it shares an executor with.
    pub fn pulse(&mut self, count: u16, half_period_us: u16) {
        let pin = self.pin as usize;
        let mask = 1u32 << pin;

        let previous = pac::IO_BANK0.gpio(pin).ctrl().read().funcsel();
        pac::SIO.gpio_out(0).value_clr().write_value(mask);
        pac::SIO.gpio_oe(0).value_set().write_value(mask);
        pac::IO_BANK0.gpio(pin).ctrl().write(|w| {
            #[cfg(feature = "rp2040")]
            w.set_funcsel(pac::io::vals::Gpio0ctrlFuncsel::SIO_0 as _);
            #[cfg(feature = "rp235x")]
            w.set_funcsel(pac::io::vals::Gpio0ctrlFuncsel::SIOB_PROC_0 as _);
        });

        for _ in 0..count {
            pac::SIO.gpio_out(0).value_set().write_value(mask);
            spin_us(half_period_us as u32);
            pac::SIO.gpio_out(0).value_clr().write_value(mask);
            spin_us(half_period_us as u32);
        }

        pac::SIO.gpio_oe(0).value_clr().write_value(mask);
        pac::IO_BANK0
            .gpio(pin)
            .ctrl()
            .write(|w| w.set_funcsel(previous));
    }

    /// Stops the state machine, empties the FIFO, reloads X, puts the program
    /// counter back on the first instruction and starts again.
    fn restart_counter(&mut self) {
        self.sm.set_enable(false);
        self.sm.clear_fifos();
        self.sm.restart();
        // `restart()` is `CTRL.SM_RESTART`, which resets the shift counters,
        // the ISR, the delay counter and any wait condition -- but not the PC.
        // Without this jump the state machine resumes wherever it stopped, and
        // where it stops is not arbitrary: a full FIFO parks it *on* an
        // `in x, 32`, which then completes the instant the FIFO is cleared and
        // stamps the freshly reloaded X. That timestamp is an edge that never
        // happened, and since the pin is low by then the program walks
        // straight through `wait_low` and stamps a second one two ticks later.
        // A pair of phantom edges at the head of a capture reads as a real
        // segment, which shifts every later segment index by one and takes the
        // measurements with it.
        unsafe {
            self.sm.exec_jmp(self.origin);
            // `mov x, ~null` — X all ones, the top of the down-counter's
            // range. Executed straight into the stopped state machine rather
            // than being an instruction in the program, so arming does not
            // have to wait for a program counter to come round to it. After
            // the jump, because an instruction written to `INSTR` executes
            // immediately without advancing the PC, so the jump's target
            // survives it.
            self.sm.exec_instr(MOV_X_NOT_NULL);
        }
        self.sm.set_enable(true);
    }

    fn start_dma(&mut self) {
        let channel = pac::DMA.ch(DMA_CHANNEL);
        channel
            .read_addr()
            .write_value(self.sm.rx_fifo_ptr() as u32);
        channel
            .write_addr()
            .write_value((&raw mut SAMPLES) as *mut u32 as u32);
        set_trans_count(channel, CAPACITY as u32);
        channel.ctrl_trig().write(|w| {
            w.set_treq_sel(PIO0_RX0_DREQ);
            w.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
            w.set_incr_read(false);
            w.set_incr_write(true);
            w.set_chain_to(DMA_CHANNEL as u8);
            w.set_en(true);
        });
    }

    fn stop_dma(&mut self) {
        pac::DMA
            .chan_abort()
            .modify(|w| w.set_chan_abort(1 << DMA_CHANNEL));
        while pac::DMA.ch(DMA_CHANNEL).ctrl_trig().read().busy() {}
    }
}

/// Transfers the channel has left to do.
///
/// A bare `u32` on the RP2040 and the low 28 bits of a two-field register on
/// the RP2350, whose other field is the transfer mode. Wrapped rather than
/// `cfg`-ed at the call site so the arithmetic that turns this into a count of
/// captured edges reads the same on both.
#[cfg(feature = "rp2040")]
fn trans_count(channel: pac::dma::Channel) -> u32 {
    channel.trans_count().read()
}

/// Transfers the channel has left to do. See the `rp2040` arm above.
#[cfg(feature = "rp235x")]
fn trans_count(channel: pac::dma::Channel) -> u32 {
    channel.trans_count().read().count()
}

/// Sets how many transfers the channel performs before it stops.
#[cfg(feature = "rp2040")]
fn set_trans_count(channel: pac::dma::Channel, count: u32) {
    channel.trans_count().write_value(count);
}

/// Sets how many transfers the channel performs before it stops.
///
/// The RP2350 packs a transfer *mode* into the top four bits of this register,
/// and mode 0 — what `default` leaves — is the RP2040's only behaviour:
/// decrement per transfer, then trigger `CHAIN_TO`. Built through the setter
/// rather than from the raw word so a count that ever grew past 28 bits would
/// be truncated rather than quietly reinterpreted as a mode.
#[cfg(feature = "rp235x")]
fn set_trans_count(channel: pac::dma::Channel, count: u32) {
    let mut value = pac::dma::regs::ChTransCount::default();
    value.set_count(count);
    channel.trans_count().write_value(value);
}

/// Busy-waits `us` microseconds against the chip's always-running 1 MHz timer.
///
/// `timerawl` is the low word of the same counter `embassy_time` is built on,
/// read directly because this runs in a synchronous request handler with no
/// executor to await on. Wrapping arithmetic rather than a comparison: the
/// counter rolls over every 71 minutes, and a naive `<` would turn that into a
/// 71-minute hang once per uptime.
fn spin_us(us: u32) {
    let start = TIMER.timerawl().read();
    while TIMER.timerawl().read().wrapping_sub(start) < us {}
}

/// `mov x, ~null`, encoded. `MOV` is opcode 0b101, then five delay/side-set
/// bits, destination X is 0b001, operation "invert" is 0b01 and source NULL
/// is 0b011.
///
/// Grouped by instruction field rather than by nibble, which is what the lint
/// below objects to and what makes the encoding checkable against the
/// datasheet's table at a glance. `embassy-rp` writes its own hand-encoded
/// instructions the same way.
#[allow(clippy::unusual_byte_groupings)]
const MOV_X_NOT_NULL: u16 = 0b101_00000_001_01_011;

/// The data request the RX FIFO of PIO0's state machine 0 raises. Named here
/// because the numbering runs PIO0 RX0..RX3 after TX0..TX3, which is easy to
/// be one out on and produces a capture of nothing rather than an error.
const PIO0_RX0_DREQ: pac::dma::vals::TreqSel = pac::dma::vals::TreqSel::PIO0_RX0;
