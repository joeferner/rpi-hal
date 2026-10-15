//! Entropy for the crypto stack, from the SoC's hardware RNG.
//!
//! The RustCrypto primitives underneath `rustls` reach for randomness
//! through `getrandom`, which has no backend for a bare-metal target and
//! refuses to compile without one. This module supplies that backend, and
//! [`fill`](crate::entropy::fill) for callers who want bytes directly.
//!
//! It is wired to the BCM2835/2836/2837's true-RNG block — a ring
//! oscillator feeding a whitener — rather than to anything derived from
//! timer jitter. That matters more here than it would on a
//! general-purpose OS: key generation, the TLS client random and the
//! ECDHE private half all come out of this one function, and there is no
//! entropy pool, no seed file carried across boots, and no other source to
//! fall back on.
//!
//! # Installed by linkage
//!
//! `getrandom::register_custom_getrandom!` defines a symbol the whole
//! program resolves against, so enabling `entropy` decides where *every*
//! crate in the binary gets its randomness — not just this one. That is
//! the same kind of program-wide choice as a `#[global_allocator]`, which
//! is why it is behind a feature that is off by default: a board with its
//! own backend leaves the feature alone and registers one, and nothing
//! here competes with it.
//!
//! # Why the generator is a static
//!
//! Constructing an [`Rng`](rpi_hal::rng::Rng) re-arms a warmup discard of
//! a quarter-million
//! samples, during which the FIFO stays empty and reads block. A fresh
//! instance per call would pay that on every random byte a handshake asks
//! for, so one is built on first use and kept.
//!
//! The warmup is worth budgeting for: it is a little under a second, and
//! it is paid inside whichever call first needs randomness rather than at
//! boot. On a board whose generator is already running it does not even
//! land on the first call — arming the discard does not flush words
//! already queued, so a handful arrive immediately and the stall follows
//! them. A board that would rather take it somewhere predictable can call
//! [`fill`](crate::entropy::fill) with a short buffer during bring-up,
//! more than once, and discard the result.
//!
//! It is built lazily rather than from a bring-up hook so that there is no
//! ordering requirement against `main`. `getrandom` can be called from
//! anywhere, including from a constructor that runs before bring-up would
//! have reached an `init` — and a backend that had to be installed first
//! would fail exactly then, in whichever dependency happened to ask
//! earliest.

use core::cell::RefCell;

use critical_section::Mutex;
use rpi_hal::rng::Rng;

/// The generator, created on first use.
static RNG: Mutex<RefCell<Option<Rng>>> = Mutex::new(RefCell::new(None));

/// Fills `dest` with hardware-generated random bytes.
///
/// Public as well as backing `getrandom`, because a caller wanting a
/// session token or a password salt should come from the same source
/// without going through a `getrandom` call that would have to unwrap an
/// error this cannot produce.
///
/// Blocks while the FIFO refills, and on the first call for the length of
/// the generator's warmup.
///
/// # Concurrency
///
/// The critical section makes concurrent callers take turns rather than
/// interleave reads of the same FIFO — two tasks each asking for 16 bytes
/// must get 16 bytes each, not 32 bytes shuffled between them. What that
/// section actually is depends on what the application built `rpi-hal`
/// with: an interrupt mask on one core, and a cross-core spinlock under
/// `multicore`, which is what it has to be for a static that a second
/// core can reach.
pub fn fill(dest: &mut [u8]) {
    critical_section::with(|cs| {
        let mut slot = RNG.borrow_ref_mut(cs);
        slot.get_or_insert_with(Rng::new).fill_bytes(dest);
    });
}

/// The `getrandom` backend.
///
/// Infallible: the hardware generator cannot fail or be unavailable, it
/// can only make a caller wait for the FIFO to refill.
fn hardware_entropy(dest: &mut [u8]) -> Result<(), getrandom::Error> {
    fill(dest);
    Ok(())
}

getrandom::register_custom_getrandom!(hardware_entropy);
