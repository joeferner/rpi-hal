//! Blocking bring-up for the BCM PCM / I2S peripheral, driving digital
//! audio *out* to an external I2S DAC (e.g. a PCM5102 / UDA1334) in
//! standard Philips-I2S master mode, DMA-fed.
//!
//! This is the digital-audio counterpart to the analog PWM audio path in
//! [`crate::pwm`]. Where PWM synthesises an analog level on a GPIO pin
//! (needing an RC filter / the board's jack circuit to recover the
//! waveform), PCM/I2S clocks the raw sample bits out over three wires —
//! bit clock, word/frame clock, and serial data — to a DAC that does the
//! conversion. Both paths share the same shape: a hardware FIFO that
//! raises a DMA request (DREQ) when it has room, fed straight from a RAM
//! buffer by the DMA controller ([`crate::dma`]) with no per-sample CPU
//! work. See "Audio" below for how the sample stream is laid out.
//!
//! # Not in the PAC
//!
//! The PCM peripheral's register block isn't modelled in `bcm2837-lpa`'s
//! SVD (only its clock generator, `CM_PCM`, is — see "Clock" below). Like
//! [`crate::unicam`] and [`crate::dma`], this driver pokes the known
//! physical registers directly. The layout and bitfields follow the
//! BCM2835 ARM Peripherals datasheet, section 8 ("PCM / I2S Audio"); the
//! block lives at ARM physical `0x3F20_3000` (VideoCore bus
//! `0x7E20_3000`).
//!
//! # Clock
//!
//! In master mode the peripheral generates the bit clock (`PCM_CLK`) and
//! frame/word clock (`PCM_FS`) itself, derived from the `CM_PCM` clock
//! generator in the clock manager. `CM_PCM` is configured here exactly
//! the way [`crate::pwm`] configures `CM_PWM`: sourced from `PLLD_per`
//! (the PAC labels the encoding `pllc()`, a naming quirk of this crate's
//! SVD — same clock the PWM path uses), integer divider only. The bit
//! clock rate is `PLLD_per / clock_divisor`, and the frame layout this
//! driver programs sends [`BITS_PER_FRAME`](crate::pcm::BITS_PER_FRAME) bit
//! clocks per stereo frame, so the sample rate falls out as
//! `bit_clock_hz / BITS_PER_FRAME`.
//! [`Pcm::clock_divisor`](crate::pcm::Pcm::clock_divisor) inverts that to pick
//! the [`Pcm::init`](crate::pcm::Pcm::init) divisor
//! for a target sample rate. As with every rate in [`crate::pwm`], the
//! result is nominal, not exact — integer divider off a PLL-derived clock
//! whose frequency (commonly cited as 500 MHz) this crate doesn't
//! independently confirm.
//!
//! # Frame format
//!
//! [`Pcm::i2s_out`](crate::pcm::Pcm::i2s_out) programs one fixed,
//! widely-compatible format:
//! standard Philips I2S, 16-bit stereo, in a 64-bit-clock frame (32 clocks
//! per channel, the data left-justified in each 32-clock slot). Concretely
//! (datasheet `MODE_A`/`TXC_A`):
//!
//! - Master for both clocks (`CLKM`/`FSM` clear) — the Pi drives `PCM_CLK`
//!   and `PCM_FS`.
//! - `CLKI` and `FSI` set: the Philips convention has the transmitter
//!   change data on the falling `PCM_CLK` edge (the DAC samples on the
//!   rising edge) and the frame sync go *low* for the left channel. These
//!   two polarity bits are the ones most likely to need flipping for a DAC
//!   that wants a different convention (e.g. left-justified); a scope on
//!   `PCM_CLK`/`PCM_FS`/`PCM_DOUT` is the way to confirm.
//! - Channel 1 (left) at frame position 1, channel 2 (right) at position
//!   33, each 16 bits wide (`CH*WID = 8`, i.e. width − 8). Position 1, not
//!   0, is the one-bit-clock delay from the frame-sync edge that Philips
//!   I2S requires.
//! - Frame packing off (`FTXP` clear): each channel's sample is one full
//!   32-bit FIFO word (the 16 data bits in the low half, sent MSB-first),
//!   so the FIFO stream is a simple interleaved `L, R, L, R, …` of one word
//!   per sample — the same buffer shape [`crate::pwm`]'s stereo audio path
//!   uses. Packing two 16-bit samples into one FIFO word (`FTXP`) would
//!   halve the data but is left as a future optimisation; at stereo 44.1
//!   kHz the unpacked rate is trivial.
//!
//! Which physical DAC pin ends up left vs right depends on the DAC, not
//! this driver: channel 1 is whatever the DAC latches while `PCM_FS` is in
//! its first (frame-sync-active) half.
//!
//! # Audio
//!
//! [`Pcm::i2s_out`](crate::pcm::Pcm::i2s_out) returns an
//! [`I2sOut`](crate::pcm::I2sOut) handle that, like
//! [`crate::pwm`]'s [`PwmAudio`](crate::pwm::PwmAudio), doesn't stream
//! samples itself: it exposes the FIFO's bus address
//! ([`I2sOut::fifo_bus_address`](crate::pcm::I2sOut::fifo_bus_address)) and the
//! TX DREQ number ([`I2sOut::dreq`](crate::pcm::I2sOut::dreq)) to hand to a
//! DMA channel
//! ([`crate::dma::Channel::write_peripheral`] for a looping buffer, or
//! [`crate::dma::Channel::stream_peripheral`] for a gapless double-buffered
//! stream). Each FIFO word is one 16-bit sample in its low 16 bits; a
//! caller building the buffer from signed PCM uses
//! [`pcm_sample`](crate::pcm::pcm_sample) and
//! interleaves left/right. Dropping the handle stops transmission and the
//! clocks.

use core::ptr::{read_volatile, write_volatile};

use crate::clock_manager;
use crate::pac::{CM_PCM, GPIO};

/// Bit clocks per stereo frame in the fixed format [`Pcm::i2s_out`]
/// programs (`FLEN + 1` — a 64-clock frame, 32 per channel). The sample
/// rate is the bit-clock rate divided by this; [`Pcm::clock_divisor`] uses
/// it to turn a target sample rate into a clock divisor.
pub const BITS_PER_FRAME: u32 = 64;

/// The DMA DREQ (pacing) number for the PCM transmit FIFO, passed to
/// [`crate::dma::Channel::write_peripheral`] so the DMA engine only pushes
/// a sample when the FIFO has room. Fixed by the SoC (DREQ 2 = PCM TX).
pub const TX_DREQ: u8 = 2;

/// VideoCore *bus* address of the PCM FIFO data register (`FIFO_A`), the
/// fixed destination a DMA channel streams samples into — pass it as
/// `dest_bus` to [`crate::dma::Channel::write_peripheral`]. This is the PCM
/// block's bus base `0x7E20_3000` plus `FIFO_A`'s `0x04` offset; it's the
/// bus alias of ARM physical `0x3F20_3004`, the address a bus master (the
/// DMA engine) must use rather than the ARM physical one.
pub const FIFO_BUS_ADDRESS: u32 = 0x7e20_3004;

/// PCM peripheral base, ARM physical (the bus alias `0x7E20_3000` is what
/// the DMA engine sees — see [`FIFO_BUS_ADDRESS`]).
const PCM_BASE: usize = 0x3f20_3000;
const CS_A: *mut u32 = PCM_BASE as *mut u32;
const FIFO_A: *mut u32 = (PCM_BASE + 0x04) as *mut u32;
const MODE_A: *mut u32 = (PCM_BASE + 0x08) as *mut u32;
const TXC_A: *mut u32 = (PCM_BASE + 0x10) as *mut u32;
const DREQ_A: *mut u32 = (PCM_BASE + 0x14) as *mut u32;

// CS_A (Control and Status) bits.
/// Enable the PCM block.
const CS_EN: u32 = 1 << 0;
/// Enable transmission (starts clocking the TX FIFO out).
const CS_TXON: u32 = 1 << 2;
/// Clear the TX FIFO (self-clearing).
const CS_TXCLR: u32 = 1 << 3;
/// Enable the DMA request interface (drive the TX DREQ).
const CS_DMAEN: u32 = 1 << 9;
/// TX FIFO error (underflow) — write 1 to clear.
const CS_TXERR: u32 = 1 << 15;
/// TX FIFO can accept data (not full).
const CS_TXD: u32 = 1 << 19;

// MODE_A bits.
/// Frame sync invert — frame sync goes low (not high) to mark the frame,
/// the Philips-I2S convention.
const MODE_FSI: u32 = 1 << 20;
/// Clock invert — outputs change on the falling clock edge, the
/// Philips-I2S convention.
const MODE_CLKI: u32 = 1 << 22;
/// Shift for `FLEN` (frame length − 1, in bit clocks).
const MODE_FLEN_SHIFT: u32 = 10;

// TXC_A bits.
/// Channel-2 (right) enable.
const TXC_CH2EN: u32 = 1 << 14;
/// Shift for the channel-2 sample position (in bit clocks from frame
/// start).
const TXC_CH2POS_SHIFT: u32 = 4;
/// Channel-1 (left) enable.
const TXC_CH1EN: u32 = 1 << 30;
/// Shift for the channel-1 sample position.
const TXC_CH1POS_SHIFT: u32 = 20;
/// Shift for the channel-1 width field (`width − 8`).
const TXC_CH1WID_SHIFT: u32 = 16;

// DREQ_A bits: the FIFO fill levels at which the TX DREQ and the (more
// urgent) TX PANIC assert. Generous mid-FIFO thresholds — the 64-entry
// FIFO leaves ample slack for the DMA engine to keep it topped up.
/// Shift for the TX DREQ threshold.
const DREQ_TX_SHIFT: u32 = 8;
/// Shift for the TX PANIC threshold.
const DREQ_TX_PANIC_SHIFT: u32 = 24;

/// Converts a signed 16-bit PCM sample to the 32-bit FIFO word the PCM
/// transmit path expects: the sample occupies the low 16 bits (sent
/// MSB-first, sign preserved), the upper bits ignored. Unlike
/// [`crate::pwm::pcm_to_duty`] there's no rescaling — an I2S DAC consumes
/// signed PCM directly, so silence is `0` and the full `i16` span maps
/// straight through. Interleave the results left, right, left, … to build
/// the DMA source buffer (see this module's "Audio" section).
pub const fn pcm_sample(sample: i16) -> u32 {
    sample as u16 as u32
}

/// Blocking driver for the PCM / I2S peripheral in master transmit mode.
///
/// Owns [`CM_PCM`], the PCM clock generator — taking it by value is what
/// makes a second `Pcm` impossible (the PCM register block itself isn't a
/// PAC singleton, being absent from the SVD, so there's nothing else to
/// take ownership of; see this module's "Not in the PAC" note).
pub struct Pcm {
    /// The PCM clock generator, held so it stays configured and enabled
    /// for the driver's lifetime.
    _cm_pcm: CM_PCM,
}

impl Pcm {
    /// The largest divisor [`Self::init`] can actually program, since
    /// `CM_PCM`'s `DIVI` field is 12 bits. See
    /// [`crate::pwm::Pwm::MAX_CLOCK_DIVISOR`], which is the same field on the
    /// sibling generator, for what happens to a larger value.
    pub const MAX_CLOCK_DIVISOR: u16 = clock_manager::MAX_DIVISOR;

    /// The slowest bit clock available, at [`Self::MAX_CLOCK_DIVISOR`].
    ///
    /// Divided by [`BITS_PER_FRAME`] this is the slowest sample rate the
    /// peripheral can be clocked for.
    pub const MIN_CLOCK_HZ: u32 = clock_manager::MIN_CLOCK_HZ;

    /// The bit-clock rate [`Self::init`] produces for `clock_divisor`.
    ///
    /// Applies the same clamp `init` does, so it reports what the hardware
    /// will run at rather than what was asked for. Nominal, like everything
    /// derived from `PLLD_per`.
    pub const fn clock_hz(clock_divisor: u16) -> u32 {
        clock_manager::clock_hz(clock_divisor)
    }

    /// Configures `CM_PCM` to run from `PLLD_per` at (nominally)
    /// `500_000_000 / clock_divisor` Hz — the PCM bit clock — and enables
    /// it. Mirrors [`crate::pwm::Pwm::init`]'s `CM_PWM` bring-up exactly
    /// (same source, integer-divider-only, kill-then-reconfigure
    /// sequence); see that method and this module's "Clock" section for
    /// the reasoning. Doesn't touch GPIO or the PCM block itself — pin
    /// muxing and peripheral setup are deferred to [`Self::i2s_out`].
    ///
    /// Kills any clock already running on `CM_PCM` first (the datasheet
    /// requires disabling the generator before changing its source or
    /// divisor, and GPU firmware may have left it enabled), and waits for
    /// `BUSY` to assert before returning so the clock is genuinely ticking
    /// by the time [`Self::i2s_out`] enables the peripheral.
    ///
    /// **`clock_divisor` is clamped to [`Self::MAX_CLOCK_DIVISOR`]**, which
    /// is 4095 rather than the 65535 the `u16` suggests — see that constant.
    /// [`Self::clock_hz`] reports what will actually be programmed.
    pub fn init(cm_pcm: CM_PCM, clock_divisor: u16) -> Self {
        cm_pcm.cs().write(|w| w.kill().set_bit().passwd().passwd());
        while cm_pcm.cs().read().busy().bit_is_set() {}

        unsafe {
            cm_pcm.div().write(|w| {
                w.divi().bits(clock_manager::clamp_divisor(clock_divisor));
                w.divf().bits(0);
                w.passwd().passwd()
            });
        }
        unsafe {
            cm_pcm.cs().write(|w| {
                w.src().pllc();
                w.mash().bits(0);
                w.passwd().passwd()
            });
        }
        cm_pcm.cs().write(|w| {
            w.src().pllc();
            w.enab().set_bit();
            w.passwd().passwd()
        });
        while cm_pcm.cs().read().busy().bit_is_clear() {}

        Self { _cm_pcm: cm_pcm }
    }

    /// Picks the [`Self::init`] `clock_divisor` that yields (nominally) a
    /// `sample_rate`-Hz stereo I2S stream, inverting the
    /// `sample_rate = bit_clock_hz / BITS_PER_FRAME` relationship this
    /// module's "Clock" section describes: `divisor = 500_000_000 /
    /// (sample_rate * BITS_PER_FRAME)`, using `PLLD_per`'s nominal 500 MHz.
    ///
    /// Integer division makes the result — and therefore the real sample
    /// rate — approximate, not exact (same caveat as
    /// [`crate::pwm::Pwm::audio_clock_divisor`]).
    ///
    /// Clamped to the range [`Self::init`] can program, `1` to
    /// [`Self::MAX_CLOCK_DIVISOR`] — the upper end for the reason that
    /// constant describes, the lower so a too-high `sample_rate` cannot yield
    /// a zero divisor.
    ///
    /// The upper clamp bites below roughly 1.9 kHz
    /// (`MIN_CLOCK_HZ / BITS_PER_FRAME`), which is under any sample rate this
    /// is likely to be asked for but is a real floor rather than a rounding
    /// concern. [`Self::clock_hz`] reports the bit clock that will result.
    pub const fn clock_divisor(sample_rate: u32) -> u16 {
        let product = sample_rate as u64 * BITS_PER_FRAME as u64;
        if product == 0 {
            return clock_manager::MIN_DIVISOR;
        }
        let divisor = clock_manager::SOURCE_HZ as u64 / product;
        if divisor > u16::MAX as u64 {
            clock_manager::MAX_DIVISOR
        } else {
            clock_manager::clamp_divisor(divisor as u16)
        }
    }

    /// Routes the I2S pins, programs the fixed Philips-I2S 16-bit-stereo
    /// frame format (see this module's "Frame format" section), enables the
    /// peripheral and its DMA interface, and starts transmission —
    /// returning an [`I2sOut`] handle to pair with a DMA channel.
    ///
    /// Uses `PCM_CLK` on GPIO18, `PCM_FS` on GPIO19, and `PCM_DOUT` on
    /// GPIO21 (all ALT0 — the I2S pins on the 40-pin header). `PCM_DIN`
    /// (GPIO20) is left alone: this is a transmit-only path. Unlike
    /// [`crate::pwm`]'s two-pins-per-channel layout, I2S has one header pin
    /// set, so the pins are fixed rather than selectable — the same
    /// approach [`crate::spi`]/[`crate::i2c`] take.
    ///
    /// The caller owns the actual streaming: feed
    /// [`I2sOut::fifo_bus_address`] and [`I2sOut::dreq`] to a DMA channel
    /// ([`crate::dma::Channel::write_peripheral`] /
    /// [`crate::dma::Channel::stream_peripheral`]). Before enabling
    /// transmission this primes the FIFO with silence (zero samples) so the
    /// frame counter starts aligned and the first frames don't underflow
    /// while the DMA engine is still spinning up.
    pub fn i2s_out(&self, gpio: &GPIO) -> I2sOut<'_> {
        // PCM_CLK / PCM_FS / PCM_DOUT — ALT0 on GPIO18/19/21. GPIO20
        // (PCM_DIN) is deliberately untouched; this is a TX-only path.
        gpio.gpfsel1().modify(|_, w| w.fsel18().pcm_clk());
        gpio.gpfsel1().modify(|_, w| w.fsel19().pcm_fs());
        gpio.gpfsel2().modify(|_, w| w.fsel21().pcm_dout());

        unsafe {
            // Start from a known-clear state: disabling the block first
            // means a warm reboot (e.g. `rpi-loader` jumping to a freshly
            // loaded kernel without a power cycle) can't leave a previous
            // run's mode/enable bits latched — the same warm-reboot concern
            // `crate::pwm` documents for its `CTL` bits. Nothing this crate
            // controls resets these registers.
            write_volatile(CS_A, 0);
            settle_delay();

            // 64-bit-clock frame: FLEN = 63, FSLEN = 32 (frame sync high --
            // or low, since FSI is set -- for the first 32 clocks, the left
            // channel). Philips polarity via FSI/CLKI. Master (CLKM/FSM
            // clear).
            let flen: u32 = BITS_PER_FRAME - 1;
            let fslen: u32 = BITS_PER_FRAME / 2;
            write_volatile(
                MODE_A,
                MODE_FSI | MODE_CLKI | (flen << MODE_FLEN_SHIFT) | fslen,
            );

            // Channel 1 (left) at position 1 (the one-clock I2S delay),
            // channel 2 (right) at position 33; both 16 bits wide
            // (width − 8 = 8).
            let ch1pos: u32 = 1;
            let ch2pos: u32 = BITS_PER_FRAME / 2 + 1;
            let width_field: u32 = 16 - 8;
            write_volatile(
                TXC_A,
                TXC_CH1EN
                    | (ch1pos << TXC_CH1POS_SHIFT)
                    | (width_field << TXC_CH1WID_SHIFT)
                    | TXC_CH2EN
                    | (ch2pos << TXC_CH2POS_SHIFT)
                    | width_field,
            );

            // Raise the TX DREQ at a mid-FIFO fill and PANIC lower, so the
            // DMA engine keeps the 64-entry FIFO topped up with slack.
            write_volatile(
                DREQ_A,
                (0x10 << DREQ_TX_PANIC_SHIFT) | (0x30 << DREQ_TX_SHIFT),
            );

            // Enable the block, then clear the TX FIFO and any stale error
            // flag. TXCLR is self-clearing; give it a moment to sync.
            write_volatile(CS_A, CS_EN);
            settle_delay();
            write_volatile(CS_A, CS_EN | CS_TXCLR | CS_TXERR);
            settle_delay();

            // Prime the FIFO with silence before transmission starts so the
            // frame counter latches onto a known (zero) sample rather than
            // underflowing while the DMA engine spins up — an underflow at
            // the very first frame can leave left/right permanently
            // swapped. Bounded so a wedged TXD can't spin forever (the FIFO
            // is 64 entries).
            let mut primed = 0;
            while read_volatile(CS_A) & CS_TXD != 0 && primed < 64 {
                write_volatile(FIFO_A, 0);
                primed += 1;
            }

            // Turn on the DMA interface and start transmission. From here
            // the caller's DMA channel keeps the FIFO fed off the TX DREQ.
            write_volatile(CS_A, CS_EN | CS_DMAEN | CS_TXON);
        }

        I2sOut { _pcm: self }
    }
}

/// Busy-wait for the short settling gaps the PCM enable sequence needs
/// (after enabling the block, and after a self-clearing `TXCLR`, the
/// datasheet wants a couple of `PCM_CLK` periods before proceeding). Not
/// calibrated against a real time base — no [`crate::timer::Timer`]
/// reference is available here — just a generously long instruction count;
/// the true minimum hasn't been characterised. Much shorter than
/// [`crate::pwm`]'s settle loop, which is covering a different (counter
/// start-up) hazard.
fn settle_delay() {
    for _ in 0..100_000u32 {
        unsafe { core::arch::asm!("nop") };
    }
}

/// A live handle to the PCM peripheral configured for DMA-fed I2S stereo
/// output, borrowed from [`Pcm`] — see [`Pcm::i2s_out`].
///
/// Like [`crate::pwm::PwmAudio`], it doesn't stream samples itself; it
/// exposes the FIFO destination and DREQ number a DMA channel needs
/// ([`crate::dma::Channel::write_peripheral`]). Dropping it stops
/// transmission and tears the setup back down (see [`Drop`]).
pub struct I2sOut<'a> {
    /// Borrows the [`Pcm`] so its clock can't be dropped (which would stop
    /// `PCM_CLK`) while this handle — and the DMA stream feeding it — is
    /// still live.
    _pcm: &'a Pcm,
}

impl I2sOut<'_> {
    /// The bus address of the PCM TX FIFO, the fixed DMA destination for
    /// samples — the value of [`FIFO_BUS_ADDRESS`], offered here so the
    /// handle carries everything the DMA side needs.
    pub fn fifo_bus_address(&self) -> u32 {
        FIFO_BUS_ADDRESS
    }

    /// The PCM transmit DREQ number, which paces the transfer — the value
    /// of [`TX_DREQ`].
    pub fn dreq(&self) -> u8 {
        TX_DREQ
    }
}

impl Drop for I2sOut<'_> {
    /// Stops transmission and the DMA interface and disables the block, so
    /// the FIFO stops draining, the clocks stop, and no further DREQs are
    /// raised once playback is done.
    fn drop(&mut self) {
        // Clearing EN stops the clocks and transmission in one write; there
        // are no other channels or shared state to preserve (unlike
        // `crate::pwm`'s two-channel `CTL`), so a full write is fine.
        unsafe {
            write_volatile(CS_A, 0);
        }
    }
}
