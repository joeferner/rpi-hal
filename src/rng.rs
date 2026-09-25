//! Blocking driver for the hardware random number generator.
//!
//! BCM2835/2836/2837 has a dedicated true-RNG block (a free-running
//! ring-oscillator entropy source feeding a whitener and a small output
//! FIFO), distinct from anything derivable in software from timing
//! jitter. Once enabled it fills a FIFO with 32-bit words at hardware
//! speed; a read pops one word and the block refills it. The upper byte
//! of the status register reports how many words are currently
//! available, so reads poll that rather than an interrupt — this is a
//! blocking, poll-only driver.
//!
//! The peripheral isn't modeled in `bcm2837-lpa`'s SVD at all, so this
//! pokes its known physical addresses directly rather than going
//! through the PAC — the same approach `uart.rs`/`sd.rs` take for the
//! legacy GPIO pull registers that SVD also omits. Register layout and
//! the enable/warmup sequence follow the BCM2835 ARM Peripherals
//! behaviour used by Linux's `bcm2835-rng` driver and the widely
//! mirrored bare-metal Pi references (bztsrc's `raspi3-tutorial`
//! `rand.c`).

/// RNG peripheral base address: the peripheral base plus the block's
/// `0x0010_4000` offset.
///
/// Taken from [`crate::soc`] rather than written out. It was the
/// BCM2836/2837 value with no chip selection, which on a BCM2835 is an
/// address the MMU has not mapped — so this driver did not return poor
/// random numbers there, it faulted.
const RNG_BASE: usize = crate::soc::PERIPHERAL_BASE as usize + 0x0010_4000;
/// Control register: bit 0 (`EN`) enables the generator.
const RNG_CTRL: *mut u32 = RNG_BASE as *mut u32;
/// Status register. Bits [31:24] hold the count of 32-bit words
/// currently available in the output FIFO; writing it sets the warmup
/// count (see [`RNG_WARMUP_COUNT`]).
const RNG_STATUS: *mut u32 = (RNG_BASE + 0x04) as *mut u32;
/// Data register: reading pops one 32-bit random word from the FIFO.
const RNG_DATA: *mut u32 = (RNG_BASE + 0x08) as *mut u32;
/// Interrupt mask register: bit 0 (`INT_OFF`) masks the FIFO interrupt.
const RNG_INT_MASK: *mut u32 = (RNG_BASE + 0x10) as *mut u32;

/// `RNG_CTRL` enable bit.
const RNG_CTRL_EN: u32 = 0x1;
/// `RNG_INT_MASK` bit that masks the block's interrupt. This driver
/// polls the status register, so the interrupt is masked to keep the
/// block from asserting an unhandled IRQ line.
const RNG_INT_OFF: u32 = 0x1;
/// Number of entropy-source samples the block discards before its
/// output is trusted, written into `RNG_STATUS` at init. Until this
/// many samples have been drawn the block presents no new words (status
/// word-count reads zero), so once [`Rng::new`] has emptied whatever was
/// already queued, the read path's own wait naturally covers warmup — no
/// separate spin in the constructor. `0x40000` is the value the
/// reference drivers use.
const RNG_WARMUP_COUNT: u32 = 0x40000;

/// Blocking driver for the hardware random number generator.
pub struct Rng {
    _private: (),
}

impl Rng {
    /// Enables the generator, arms its warmup discard, and empties the
    /// output FIFO.
    ///
    /// Doesn't block: warmup completes asynchronously in hardware, and
    /// the block simply reports no words available until it's done — so
    /// the first [`next_u32`](Rng::next_u32) transparently waits out any
    /// remaining warmup rather than this constructor stalling for it.
    ///
    /// The FIFO drain is what makes that true on a board whose generator
    /// was already running — left enabled by the firmware, or by a
    /// previous image loaded over a UART loader, which is the normal case
    /// when developing without power-cycling. Arming a fresh warmup count
    /// does not flush words the block had already queued, so without the
    /// drain the first few reads return those and the warmup stall lands
    /// at an unpredictable later word instead. Those queued words may be
    /// perfectly good output from the generator's previous run, but
    /// nothing readable from this side distinguishes that from a block
    /// enabled with no warmup armed at all, whose first words are exactly
    /// the biased samples the discard exists to remove — so they go.
    ///
    /// # Safety of construction
    ///
    /// Unlike this crate's PAC-backed drivers, there's no singleton
    /// token to hand over (the peripheral isn't in `bcm2837-lpa`), so
    /// nothing here prevents constructing two `Rng`s aliasing the same
    /// hardware. That's benign — the block is a shared read-only entropy
    /// source with no per-instance state to corrupt; two readers just
    /// draw independent words from the same FIFO — so `new` is safe. If
    /// exclusive ownership matters to a caller, they hold the single
    /// `Rng` themselves.
    pub fn new() -> Self {
        unsafe {
            // Discard the first RNG_WARMUP_COUNT samples before any
            // word is presented as available.
            core::ptr::write_volatile(RNG_STATUS, RNG_WARMUP_COUNT);
            // Mask the FIFO interrupt: this driver polls.
            let mask = core::ptr::read_volatile(RNG_INT_MASK) | RNG_INT_OFF;
            core::ptr::write_volatile(RNG_INT_MASK, mask);
            // Enable the generator.
            core::ptr::write_volatile(RNG_CTRL, RNG_CTRL_EN);
        }
        let rng = Self { _private: () };
        // Drop anything the block had queued from a previous run, so the
        // warmup just armed is what every word this `Rng` hands out has
        // been through. On a cold boot the FIFO is empty and this does
        // not execute; the block is now warming up, so it presents no
        // new words for the loop to chase.
        while rng.words_available() != 0 {
            unsafe { core::ptr::read_volatile(RNG_DATA) };
        }
        rng
    }

    /// Number of 32-bit words currently available to read without
    /// blocking (`RNG_STATUS` bits [31:24]).
    fn words_available(&self) -> u32 {
        unsafe { core::ptr::read_volatile(RNG_STATUS) >> 24 }
    }

    /// Blocks until a random word is available, then reads and returns
    /// it.
    pub fn next_u32(&mut self) -> u32 {
        while self.words_available() == 0 {
            core::hint::spin_loop();
        }
        unsafe { core::ptr::read_volatile(RNG_DATA) }
    }

    /// Non-blocking sibling of [`next_u32`](Rng::next_u32): returns
    /// `None` if the FIFO is currently empty, otherwise pops and returns
    /// one word.
    pub fn try_next_u32(&mut self) -> Option<u32> {
        if self.words_available() == 0 {
            return None;
        }
        Some(unsafe { core::ptr::read_volatile(RNG_DATA) })
    }

    /// Blocks for two words and returns them as one 64-bit value.
    pub fn next_u64(&mut self) -> u64 {
        let lo = self.next_u32() as u64;
        let hi = self.next_u32() as u64;
        (hi << 32) | lo
    }

    /// Fills `dest` with random bytes, blocking as needed.
    ///
    /// Draws full 32-bit words and copies out little-endian; a trailing
    /// partial word (1-3 bytes) draws one more word and uses only its
    /// low bytes, discarding the rest.
    pub fn fill_bytes(&mut self, dest: &mut [u8]) {
        let mut offset = 0;
        while offset < dest.len() {
            let word = self.next_u32().to_le_bytes();
            let n = (dest.len() - offset).min(4);
            dest[offset..offset + n].copy_from_slice(&word[..n]);
            offset += n;
        }
    }
}

impl Default for Rng {
    /// Equivalent to [`Rng::new`].
    fn default() -> Self {
        Self::new()
    }
}
