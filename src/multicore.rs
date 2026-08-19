//! Bringing up secondary cores (1-3).
//!
//! Out of reset, the GPU firmware does *not* release cores 1-3 to the
//! kernel's entry point the way it does core 0. Instead it leaves each of
//! them parked in its own stub until explicitly woken, and the wake
//! mechanism differs by execution state:
//!
//! - **AArch32**: each core watches its ARM-local *mailbox 3* register;
//!   `launch` hands off the stack pointers and entry through that core's
//!   mailboxes (Device memory, coherent between cores with no cache
//!   maintenance) and wakes it by writing mailbox 3. (Confirmed on
//!   hardware: a secondary core made to record its arrival from our own
//!   `_start` never did -- it never reaches `_start` at all.)
//! - **AArch64**: the firmware's `armstub8` parks cores in a *spin-table*
//!   -- each polls a fixed low address (`0xd8 + 8*core`) and jumps to
//!   whatever 64-bit address is written there, once woken by `sev`. The
//!   spin-table carries only the entry address, so `launch` passes the
//!   stack + entry through `SECONDARY_PARAMS` in shared RAM instead,
//!   cache-cleaning it (the target core reads with its caches still off)
//!   before writing the core's slot with the address of
//!   `__secondary_core_entry` (the secondary64.s trampoline).
//!
//! Either way the trampoline reads back the handoff, brings up the core's
//! own MMU/caches ([`rpi_hal_mmu_init`](crate)), and jumps to `entry`.
//!
//! Requires the `mmu` feature (pulled in automatically -- see this crate's
//! `Cargo.toml`): the secondary core's own `rpi_hal_mmu_init` call is what
//! gives it Normal/Cacheable/Shareable RAM so `core::sync::atomic` (and
//! hence `critical_section`) work once it's running alongside core 0.
//!
//! Code that ends up running on more than one core -- the entry functions
//! here, but also the single `__irq_handler` and panic handler all of them
//! share -- can ask which one it is on with [`crate::cpu::core_id`]. That
//! lives outside this module on purpose, so it can be read without this
//! feature.

#[cfg(target_arch = "aarch64")]
core::arch::global_asm!(include_str!("secondary64.s"));

/// Must match `boot.s`'s `.equ IRQ_STACK_SIZE` -- both sides carve the
/// same fixed region off the top of a core's stack for IRQ-mode use. Only
/// meaningful on AArch32; AArch64 takes exceptions on the single SP_EL1
/// and uses the whole stack.
const IRQ_STACK_SIZE: usize = 0x1000;

// ---- AArch32: ARM-local mailbox handoff ----------------------------------

/// Base of the ARM-local mailbox *write-set* registers. Each core owns
/// four consecutive 32-bit mailboxes starting here, with a `0x10` stride
/// between cores, so core `C`'s mailbox `M` set register is at
/// `MAILBOX_SET_BASE + 0x10 * C + 4 * M`. `boot.s`'s
/// `__secondary_core_entry` reads them back from the matching *read*
/// registers at `0x4000_00C0` (same layout, `0x40` higher).
#[cfg(target_arch = "arm")]
const MAILBOX_SET_BASE: usize = 0x4000_0080;

#[cfg(target_arch = "arm")]
extern "C" {
    /// The `boot.s` trampoline a released secondary core starts
    /// executing -- see this module's doc comment. Only its *address* is
    /// ever used (written into mailbox 3); it is never called from Rust.
    fn __secondary_core_entry() -> !;
}

/// Hands off the stack pointers and `entry` to core `core_id` (1-3)
/// through its ARM-local mailboxes, then wakes it. Mailbox 3 is written
/// last, since writing it is what the firmware stub is watching for;
/// `dsb` orders all four writes ahead of the `sev`, and `sev` wakes the
/// stub out of its `wfe`.
#[cfg(target_arch = "arm")]
fn launch<const BYTES: usize>(
    core_id: usize,
    stack: &'static mut Stack<BYTES>,
    entry: extern "C" fn() -> !,
) {
    use core::ptr::write_volatile;

    let sp_irq = stack.top();
    let sp_main = sp_irq - IRQ_STACK_SIZE;
    let mailbox = MAILBOX_SET_BASE + 0x10 * core_id;

    // SAFETY: these are the Device-mapped ARM-local mailbox registers for
    // a real core (1-3); a fresh mailbox reads 0, so writing the full
    // value through the write-set register lands it exactly. `dsb`/`sev`
    // are plain side-effect-only barriers.
    unsafe {
        write_volatile((mailbox) as *mut u32, sp_main as u32);
        write_volatile((mailbox + 0x4) as *mut u32, sp_irq as u32);
        write_volatile((mailbox + 0x8) as *mut u32, entry as usize as u32);
        write_volatile(
            (mailbox + 0xC) as *mut u32,
            __secondary_core_entry as *const () as u32,
        );
        core::arch::asm!("dsb", "sev");
    }
}

// ---- AArch64: armstub8 spin-table handoff --------------------------------

/// `armstub8` spin-table base: `spin_cpu0` sits at `0xd8` and core `C`
/// polls `0xd8 + 8 * C`, jumping to the 64-bit address written there once
/// released by `sev`.
#[cfg(target_arch = "aarch64")]
const SPIN_TABLE_BASE: usize = 0xd8;

#[cfg(target_arch = "aarch64")]
extern "C" {
    /// The secondary64.s trampoline a released core jumps to; its address
    /// is written into the core's spin-table slot.
    fn __secondary_core_entry() -> !;

    /// `{ sp, entry }` (two `u64`s) per core, indexed by core id -- the
    /// handoff the spin-table can't carry itself. Defined in secondary64.s.
    static SECONDARY_PARAMS: [u64; 8];
}

/// Writes `entry` and the core's stack into [`SECONDARY_PARAMS`], cleans
/// those cache lines (the target core reads them with its caches still
/// off), then releases the core by writing the trampoline's address into
/// its spin-table slot and issuing `sev`. `clean_range`'s trailing `dsb`
/// orders each write ahead of the wake.
#[cfg(target_arch = "aarch64")]
fn launch<const BYTES: usize>(
    core_id: usize,
    stack: &'static mut Stack<BYTES>,
    entry: extern "C" fn() -> !,
) {
    use core::ptr::write_volatile;

    // AArch64 has no separate banked IRQ stack (exceptions to EL1h run on
    // SP_EL1), so the whole stack is the core's single stack. Mask to 16
    // bytes for the AArch64 stack-pointer alignment requirement.
    let sp = (stack.top() as u64) & !0xF;

    // SAFETY: SECONDARY_PARAMS is a fixed 4-core buffer defined in
    // secondary64.s; core_id is 1..=3, so the slot is in bounds. The
    // spin-table slot is the firmware's own release word for this core.
    unsafe {
        let params = core::ptr::addr_of!(SECONDARY_PARAMS) as *mut u64;
        params.add(2 * core_id).write_volatile(sp);
        params
            .add(2 * core_id + 1)
            .write_volatile(entry as usize as u64);
        crate::cache::clean_range(params.add(2 * core_id) as u32, 16);

        let slot = (SPIN_TABLE_BASE + 8 * core_id) as *mut u64;
        write_volatile(slot, __secondary_core_entry as *const () as u64);
        crate::cache::clean_range(slot as u32, 8);

        core::arch::asm!("sev");
    }
}

// ---- Shared API ----------------------------------------------------------

/// A statically-allocated stack for a secondary core.
///
/// 8-byte aligned, since AAPCS requires the stack pointer to be 8-byte
/// aligned at public interfaces. On AArch32 `launch` carves the top
/// `IRQ_STACK_SIZE` bytes off for IRQ-mode use (mirroring how core 0's
/// own stack is laid out in `boot.s`) and hands the rest to the core's
/// main (SVC) mode; on AArch64 the whole stack is used. The size is
/// enforced to fit at compile time, not checked at runtime.
#[repr(C, align(8))]
pub struct Stack<const BYTES: usize> {
    mem: [u8; BYTES],
}

impl<const BYTES: usize> Stack<BYTES> {
    /// Compile-time check that `BYTES` is a valid stack size -- evaluated
    /// (forced) from `Stack::new`, so an invalid `BYTES` fails to build
    /// rather than misbehaving at runtime.
    const VALID_SIZE: () = assert!(
        BYTES.is_multiple_of(8) && BYTES > IRQ_STACK_SIZE,
        "Stack size must be a multiple of 8 and larger than the reserved IRQ-mode region"
    );

    /// A new, zeroed stack.
    pub const fn new() -> Self {
        const { Self::VALID_SIZE };
        Self { mem: [0; BYTES] }
    }

    /// Address one past the last byte -- where a full-descending stack
    /// starts.
    fn top(&mut self) -> usize {
        self.mem.as_mut_ptr() as usize + BYTES
    }
}

impl<const BYTES: usize> Default for Stack<BYTES> {
    /// Same as [`Stack::new`] -- a new, zeroed stack.
    fn default() -> Self {
        Self::new()
    }
}

/// Grants access to secondary cores 1-3, each of which the firmware leaves
/// parked until [`spawn`](Core1::spawn) wakes it -- see this module's doc
/// comment.
///
/// The `unsafe`-once-only contract on [`steal`](Cores::steal) plus each
/// `CoreN::spawn` consuming `self` is what prevents double-launching a
/// core -- the same ownership-based singleton idiom this crate already
/// uses for `pac::Peripherals::steal`, rather than a runtime "already
/// launched" check.
pub struct Cores {
    /// Secondary core 1.
    pub core1: Core1,
    /// Secondary core 2.
    pub core2: Core2,
    /// Secondary core 3.
    pub core3: Core3,
}

impl Cores {
    /// Steals ownership of all three secondary cores.
    ///
    /// # Safety
    ///
    /// Must be called at most once -- calling it twice hands out two
    /// `Core1`s (etc.) that could both call `spawn`, racing to launch the
    /// same physical core. Mirrors `pac::Peripherals::steal`.
    pub unsafe fn steal() -> Self {
        Self {
            core1: Core1(()),
            core2: Core2(()),
            core3: Core3(()),
        }
    }
}

/// Handle to secondary core 1. See [`Cores`].
pub struct Core1(());

/// Handle to secondary core 2. See [`Cores`].
pub struct Core2(());

/// Handle to secondary core 3. See [`Cores`].
pub struct Core3(());

macro_rules! impl_spawn {
    ($core:ident, $core_id:expr) => {
        impl $core {
            /// Starts `entry` running on this core, using `stack` as its
            /// stack.
            ///
            /// # Safety
            ///
            /// `entry` must never return (already enforced by its `-> !`
            /// type) and must be safe to run concurrently with whatever
            /// core 0 (or any other spawned core) does with shared state
            /// from this point on -- the type system can't check that part.
            pub unsafe fn spawn<const BYTES: usize>(
                self,
                stack: &'static mut Stack<BYTES>,
                entry: extern "C" fn() -> !,
            ) {
                launch($core_id, stack, entry);
            }
        }
    };
}

impl_spawn!(Core1, 1);
impl_spawn!(Core2, 2);
impl_spawn!(Core3, 3);

// ---- Sharing data between cores ------------------------------------------

/// Bytes in a cache line on this SoC's cores — which is also the size of
/// the *exclusives reservation granule*, the unit an exclusive monitor
/// reserves.
///
/// Both the Cortex-A7 and the Cortex-A53 use 64-byte lines. This is the
/// *true* size, deliberately unlike the conservative lower bound the
/// cache-maintenance helpers stride by: a stride smaller than the line is
/// harmless there, whereas padding smaller than the line is exactly the
/// bug [`CacheAligned`] exists to prevent.
pub const CACHE_LINE_BYTES: usize = 64;

/// Pads `T` out to a whole cache line, so that stores to it cannot disturb
/// an exclusive reservation held on whatever happens to be laid out beside
/// it.
///
/// **Wrap any atomic that two cores touch.** An exclusive monitor reserves
/// a [granule](CACHE_LINE_BYTES), not an address, so a store by one core to
/// *anywhere* in that granule clears another core's reservation: two
/// unrelated atomics that share a line are not independent, and each core's
/// `compare_exchange` or `swap` makes the other's fail and retry. Two cores
/// working on neighbouring atomics can starve each other indefinitely —
/// and since the clearing of an exclusive monitor is itself a `wfe`
/// wake-up event, a core idling in `wfe` is woken by the very store whose
/// reservation its own next store will knock down.
///
/// The other half of that bargain belongs to the waiting side: a core that
/// is waiting rather than working should poll with a plain load and attempt
/// an exclusive only once the lock looks free (test-and-test-and-set, which
/// is what the `spin` crate's mutexes do), so that waiting costs the other
/// cores nothing.
///
/// ```no_run
/// use core::sync::atomic::AtomicBool;
/// use rpi_hal::multicore::CacheAligned;
///
/// /// Set by core 0, polled by core 1.
/// static READY: CacheAligned<AtomicBool> = CacheAligned(AtomicBool::new(false));
/// ```
#[repr(align(64))]
pub struct CacheAligned<T>(
    /// The value being kept to itself.
    pub T,
);

const _: () = assert!(
    core::mem::align_of::<CacheAligned<u8>>() == CACHE_LINE_BYTES,
    "CacheAligned's repr(align) has to match CACHE_LINE_BYTES"
);

/// Wakes every core waiting in [`wait_for_event`] — a `dsb` followed by
/// `sev`.
///
/// The other half of a cross-core handoff: publish the data with a release
/// store, then call this so a core that has nothing to do until then can
/// wait instead of spinning.
///
/// The `dsb` is not decoration. A release store is *ordered* before the
/// `sev`, but ordering is not completion: the store can still be in flight
/// when the event is broadcast, and the woken core — sitting in `wfe`, a
/// few cycles from its next instruction — then loads the old value, goes
/// back to sleep, and is never woken again if the producer's next move
/// depends on seeing the flag consumed. `dsb` waits for the store to be
/// observable before the wake goes out, which is why
/// [`spawn`](Core1::spawn)'s own handoff pairs the two the same way.
///
/// Waking a core that isn't waiting is harmless: the event is recorded in
/// each core's event register, so its next [`wait_for_event`] returns
/// immediately rather than missing the wake-up. A consumer should still
/// treat a lost wake as possible and re-check its condition — see
/// [`wait_for_event`].
pub fn signal_event() {
    // SAFETY: neither instruction takes operands or touches memory; `dsb`
    // waits for outstanding accesses and `sev` sets every core's event
    // register.
    unsafe { core::arch::asm!("dsb ish", "sev") };
}

/// Waits until some core calls [`signal_event`] (the `wfe` instruction) —
/// how a core with nothing to do until another one says so should idle.
///
/// Always call this in a loop that re-checks the condition it's waiting
/// for. `wfe` returns on any event, and several things besides `sev` count
/// as one: an interrupt, a debug event, or the clearing of this core's
/// exclusive monitor by another core's store (see [`CacheAligned`]).
/// A return means "something happened", never "the thing you wanted
/// happened".
pub fn wait_for_event() {
    // SAFETY: `wfe` has no operands; at worst it returns immediately.
    unsafe { core::arch::asm!("wfe") };
}
