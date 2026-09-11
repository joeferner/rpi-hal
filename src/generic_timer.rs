//! Per-core ARM generic timer -- the architected timer built into the
//! Cortex-A7 (BCM2836) and Cortex-A53 (BCM2837) -- and the
//! interrupt-driven deadline scheduling it provides.
//!
//! Unlike the BCM System Timer ([`crate::timer`]), which is a single
//! memory-mapped counter shared by all cores and the GPU firmware, the
//! generic timer is *per-core* CPU state, read and armed through system
//! registers (CP15 coprocessor registers on AArch32, the `CNT*_EL0`
//! registers on AArch64) rather than MMIO. Each core has its own
//! comparator and its own timer interrupt, so each core can schedule and
//! take its own tick independently -- which is what an interrupt-driven
//! executor (e.g. Embassy) needs and what the polling-only, single-
//! comparator System Timer path can't give per core.
//!
//! This drives the *physical* timer (`CNTP`), the one accessible from the
//! non-secure PL1/EL1 state `rt`'s boot code leaves the core in (`boot.s`
//! drops HYP->SVC, `boot64.s` drops EL2->EL1, both non-secure). Its
//! interrupt is the non-secure physical timer IRQ (`CNTPNSIRQ`), routed to
//! the calling core through the BCM2836/2837 ARM-local interrupt
//! controller -- the *per-core* controller that the legacy `crate::lic`
//! is explicitly **not** (see that module's doc comment). Routing lives on
//! [`GenericTimer::route_irq`](crate::generic_timer::GenericTimer::route_irq)
//! here rather than in a separate controller
//! type: the generic timer is (today) the only user of that controller
//! this crate wraps, and its routing is inherently per-core, so folding it
//! in avoids a half-built shared abstraction the crate doesn't yet need.
//!
//! Three independent gates all have to be open for a tick to fire, the
//! same shape as the System Timer path:
//!
//! 1. the peripheral raising it --
//!    [`arm_after_us`](crate::generic_timer::GenericTimer::arm_after_us) /
//!    [`set_deadline`](crate::generic_timer::GenericTimer::set_deadline)
//!    program the comparator and enable the timer;
//! 2. the source routed to this core --
//!    [`route_irq`](crate::generic_timer::GenericTimer::route_irq);
//! 3. the CPU-level IRQ mask open -- [`crate::irq::enable_irq`].
//!
//! The counter is free-running from boot, but *not* at a rate that can be
//! taken on trust. `CNTFRQ` -- what
//! [`GenericTimer::frequency`](crate::generic_timer::GenericTimer::frequency)
//! reports -- is a register firmware writes; the rate the counter actually
//! advances at comes from the ARM-local prescaler, and nothing makes the two
//! agree. On a Pi 3 in AArch32 they disagree by exactly 19.2x: the firmware
//! divides the 19.2 MHz crystal down to 1 MHz while `CNTFRQ` still reads
//! 19_200_000, so a duration computed from the pair is 19.2x short. The
//! 64-bit path escapes it only because `armstub8` sets the prescaler to
//! unity.
//!
//! So [`GenericTimer::new`](crate::generic_timer::GenericTimer::new) sets the
//! prescaler to unity itself, which makes the counter run at the crystal rate
//! `CNTFRQ` claims and makes both execution states behave the same. Unlike
//! the System Timer, this one does need that much clock setup.

use core::arch::asm;

/// Base of the ARM-local "Core `N` timers interrupt control" registers:
/// core `N`'s register is at `0x4000_0040 + 4 * N`. Part of the same
/// ARM-local peripheral block `mmu.rs` device-maps for the inter-core
/// mailboxes (see [`crate::multicore`]).
const LOCAL_TIMER_IRQCTL_BASE: usize = 0x4000_0040;

/// ARM-local "Core timer prescaler". The counter advances at
/// `source * prescaler / 2^31`, where the source is the 19.2 MHz crystal
/// unless the control register selects the APB clock.
const LOCAL_TIMER_PRESCALER: usize = 0x4000_0008;

/// Prescaler value for a 1:1 divide, so the counter runs at the source
/// clock and `CNTFRQ` describes it correctly.
const PRESCALER_UNITY: u32 = 0x8000_0000;

/// `CNTPNSIRQ` (non-secure physical timer) IRQ-enable bit in a core's
/// "timers interrupt control" register -- the physical timer this module
/// drives, in the non-secure state boot leaves the core in.
const CNTPNSIRQ: u32 = 1 << 1;

/// `CNTP_CTL.ENABLE` (bit 0): the timer comparator is running.
const CTL_ENABLE: u32 = 1 << 0;
/// `CNTP_CTL.ISTATUS` (bit 2, read-only): the timer condition is met
/// (`CNTPCT >= CNTP_CVAL`) and the interrupt is asserted. Level-sensitive:
/// it clears only when the comparator is moved back into the future, which
/// is why acking a fired tick means re-arming, not a write-1-to-clear (see
/// [`GenericTimer::arm_after_us`]). `CNTP_CTL.IMASK` (bit 1) is left 0
/// throughout -- the interrupt condition is never masked at the timer
/// itself; gating is done at the controller and CPU level instead.
const CTL_ISTATUS: u32 = 1 << 2;

/// The per-core ARM generic (architected) timer, driving the non-secure
/// physical timer (`CNTP`).
///
/// A zero-sized handle: it owns no peripheral token because the generic
/// timer is CPU state, not an MMIO peripheral, and every method acts on
/// *the core it is called from*. A `GenericTimer` obtained on core 0
/// configures core 0's timer; one obtained inside a secondary core's entry
/// point configures that core's. Re-arming from an interrupt handler (as
/// the examples do) is exactly this pattern -- construct one and act on the
/// core currently taking the interrupt.
pub struct GenericTimer {
    _private: (),
}

impl GenericTimer {
    /// Creates a handle to the calling core's generic timer.
    ///
    /// Sets the ARM-local counter prescaler to unity, so the counter advances
    /// at the rate `CNTFRQ` reports. Idempotent, and already the value a
    /// 64-bit boot arrives with; see the module docs for why a 32-bit boot
    /// does not.
    pub fn new() -> Self {
        // Make the counter tick at the rate `CNTFRQ` advertises.
        //
        // `CNTFRQ` is not wired to anything: it is a register firmware writes,
        // and nothing ties it to how fast `CNTPCT` actually advances. That
        // rate comes from the prescaler above, and the two do not have to
        // agree -- on a Pi 3 they do not. The 32-bit firmware sets the
        // prescaler to `0x06AA_AAAB`, exactly 19.2 MHz / 19.2, so the counter
        // runs at 1 MHz while `CNTFRQ` still reads 19_200_000; every duration
        // derived from the pair then comes out 19.2x short. The 64-bit path
        // does not have the problem because `armstub8` sets the prescaler to
        // unity, which is what this restores.
        //
        // Writing it here rather than trusting firmware keeps both execution
        // states identical and keeps `CNTFRQ` honest. The cost is a write to
        // a register shared by all four cores, which is idempotent and
        // already the value the 64-bit boot arrives with.
        //
        // SAFETY: an ARM-local peripheral register, device-mapped by
        // `mmu.rs` alongside the inter-core mailboxes.
        unsafe { core::ptr::write_volatile(LOCAL_TIMER_PRESCALER as *mut u32, PRESCALER_UNITY) };
        Self { _private: () }
    }

    /// The counter frequency in Hz, as programmed by the firmware
    /// (`CNTFRQ`) -- 19.2 MHz on this hardware. Used to convert between
    /// microseconds and counter ticks.
    pub fn frequency(&self) -> u32 {
        read_cntfrq()
    }

    /// The current 64-bit physical count (`CNTPCT`) -- a monotonic tick
    /// count since the counter started, at [`frequency`](Self::frequency)
    /// ticks per second. The architected read is atomic, so unlike the
    /// System Timer's split `CLO`/`CHI` this needs no re-read guard.
    pub fn now(&self) -> u64 {
        read_cntpct()
    }

    /// Busy-waits for approximately `us` microseconds against the counter.
    /// Independent of the System Timer peripheral, so a core that doesn't
    /// own `SYSTMR` can still delay.
    pub fn delay_us(&self, us: u32) {
        self.delay_ticks((us as u64 * self.frequency() as u64) / 1_000_000);
    }

    /// Busy-waits for approximately `ms` milliseconds against the counter.
    pub fn delay_ms(&self, ms: u32) {
        self.delay_ticks((ms as u64 * self.frequency() as u64) / 1_000);
    }

    /// Arms the physical timer to fire `us` microseconds from now and
    /// enables it (`CNTP_TVAL` <- `us` in ticks, `CNTP_CTL.ENABLE` <- 1).
    ///
    /// Also the way to *acknowledge* a fired tick: the interrupt is
    /// level-sensitive on `CNTP_CTL.ISTATUS`, so re-arming past the current
    /// count is what deasserts it -- there is no write-1-to-clear. Calling
    /// this from the handler both acks the tick just taken and schedules
    /// the next one.
    ///
    /// `CNTP_TVAL` is a 32-bit signed down-counter, so the interval is
    /// bounded (~111 s at 19.2 MHz); for longer or drift-free periods use
    /// [`set_deadline`](Self::set_deadline) against an absolute
    /// [`now`](Self::now)-based deadline instead.
    pub fn arm_after_us(&self, us: u32) {
        let ticks = (us as u64 * self.frequency() as u64) / 1_000_000;
        write_cntp_tval(ticks as u32);
        write_cntp_ctl(CTL_ENABLE);
    }

    /// Arms the physical timer to fire when the count reaches the absolute
    /// `deadline` (`CNTP_CVAL` <- `deadline`, in the same units as
    /// [`now`](Self::now)) and enables it.
    ///
    /// This is the drift-free primitive an executor wants: advancing the
    /// deadline by a fixed period each tick (`set_deadline(deadline +=
    /// period)`) keeps the cadence pinned to the counter rather than to
    /// when the handler happened to run, and the full 64-bit comparator
    /// removes the interval bound [`arm_after_us`](Self::arm_after_us) has.
    pub fn set_deadline(&self, deadline: u64) {
        write_cntp_cval(deadline);
        write_cntp_ctl(CTL_ENABLE);
    }

    /// True if the timer condition is currently met (`CNTP_CTL.ISTATUS`) --
    /// i.e. this timer is the source asserting an interrupt. Lets a handler
    /// tell the generic timer apart from other sources without reading the
    /// ARM-local controller's IRQ-source register.
    pub fn is_pending(&self) -> bool {
        read_cntp_ctl() & CTL_ISTATUS != 0
    }

    /// Stops the timer (`CNTP_CTL.ENABLE` <- 0). The counter keeps running
    /// -- [`now`](Self::now) is unaffected -- but no further interrupt is
    /// raised until it is armed again.
    pub fn disable(&self) {
        write_cntp_ctl(0);
    }

    /// Routes this core's non-secure physical timer IRQ (`CNTPNSIRQ`) to
    /// the core through the ARM-local interrupt controller -- the second of
    /// the three gates (see the module doc comment). Per-core: it programs
    /// the control register belonging to whichever core calls it.
    pub fn route_irq(&self) {
        modify_local_timer_ctl(|v| v | CNTPNSIRQ);
    }

    /// Masks this core's physical timer IRQ at the ARM-local interrupt
    /// controller -- the inverse of [`route_irq`](Self::route_irq).
    pub fn mask_irq(&self) {
        modify_local_timer_ctl(|v| v & !CNTPNSIRQ);
    }

    /// Busy-waits until the count advances by `ticks`.
    fn delay_ticks(&self, ticks: u64) {
        let target = self.now() + ticks;
        while self.now() < target {}
    }
}

impl Default for GenericTimer {
    /// Same as [`GenericTimer::new`].
    fn default() -> Self {
        Self::new()
    }
}

impl embedded_hal::delay::DelayNs for GenericTimer {
    /// Busy-waits `ns` nanoseconds against the counter, rounding the
    /// tick count up so a nonzero request always waits at least one tick.
    fn delay_ns(&mut self, ns: u32) {
        self.delay_ticks((ns as u64 * self.frequency() as u64).div_ceil(1_000_000_000));
    }

    /// Delegates to the inherent [`GenericTimer::delay_us`].
    fn delay_us(&mut self, us: u32) {
        GenericTimer::delay_us(self, us);
    }

    /// Delegates to the inherent [`GenericTimer::delay_ms`].
    fn delay_ms(&mut self, ms: u32) {
        GenericTimer::delay_ms(self, ms);
    }
}

/// Read-modify-writes the calling core's ARM-local "timers interrupt
/// control" register.
fn modify_local_timer_ctl(f: impl FnOnce(u32) -> u32) {
    let reg = (LOCAL_TIMER_IRQCTL_BASE + 4 * crate::cpu::core_id()) as *mut u32;
    // SAFETY: `reg` is the device-mapped ARM-local timer-IRQ control
    // register for the calling core (mapped by `mmu.rs`; a core only ever
    // addresses its own register, so the read-modify-write races with no
    // other core). Callers set routing up during single-threaded init with
    // IRQs still masked, so it also can't race this core's own handler.
    unsafe {
        let cur = core::ptr::read_volatile(reg);
        core::ptr::write_volatile(reg, f(cur));
    }
}

// ---- Architecture-specific register access -------------------------------
//
// The generic timer registers are CP15 coprocessor registers on AArch32
// and `CNT*_EL0` system registers on AArch64 -- the only part of this
// module that differs by execution state. 64-bit reads/writes are one
// MRRC/MCRR (two 32-bit halves) on AArch32 and a single MRS/MSR on
// AArch64. An `isb` precedes each counter read so it isn't satisfied
// speculatively ahead of program order, which would skew a timestamp.

/// Reads `CNTFRQ` (counter frequency, Hz).
#[cfg(target_arch = "arm")]
#[inline(always)]
fn read_cntfrq() -> u32 {
    let freq;
    unsafe { asm!("mrc p15, 0, {}, c14, c0, 0", out(reg) freq) };
    freq
}

/// Reads the 64-bit `CNTPCT` physical count.
#[cfg(target_arch = "arm")]
#[inline(always)]
fn read_cntpct() -> u64 {
    let (lo, hi): (u32, u32);
    unsafe { asm!("isb", "mrrc p15, 0, {0}, {1}, c14", out(reg) lo, out(reg) hi) };
    ((hi as u64) << 32) | lo as u64
}

/// Writes `CNTP_TVAL` (physical timer value, a signed down-counter).
#[cfg(target_arch = "arm")]
#[inline(always)]
fn write_cntp_tval(v: u32) {
    unsafe { asm!("mcr p15, 0, {}, c14, c2, 0", in(reg) v) };
}

/// Writes the 64-bit `CNTP_CVAL` physical compare value.
#[cfg(target_arch = "arm")]
#[inline(always)]
fn write_cntp_cval(v: u64) {
    let (lo, hi) = (v as u32, (v >> 32) as u32);
    unsafe { asm!("mcrr p15, 2, {0}, {1}, c14", in(reg) lo, in(reg) hi) };
}

/// Writes `CNTP_CTL` (physical timer control).
#[cfg(target_arch = "arm")]
#[inline(always)]
fn write_cntp_ctl(v: u32) {
    unsafe { asm!("mcr p15, 0, {}, c14, c2, 1", in(reg) v) };
}

/// Reads `CNTP_CTL` (physical timer control).
#[cfg(target_arch = "arm")]
#[inline(always)]
fn read_cntp_ctl() -> u32 {
    let ctl;
    unsafe { asm!("mrc p15, 0, {}, c14, c2, 1", out(reg) ctl) };
    ctl
}

/// See the AArch32 sibling above.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
fn read_cntfrq() -> u32 {
    let freq: u64;
    unsafe { asm!("mrs {}, cntfrq_el0", out(reg) freq) };
    freq as u32
}

/// See the AArch32 sibling above.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
fn read_cntpct() -> u64 {
    let count: u64;
    unsafe { asm!("isb", "mrs {}, cntpct_el0", out(reg) count) };
    count
}

/// See the AArch32 sibling above.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
fn write_cntp_tval(v: u32) {
    let v = v as u64;
    unsafe { asm!("msr cntp_tval_el0, {}", in(reg) v) };
}

/// See the AArch32 sibling above.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
fn write_cntp_cval(v: u64) {
    unsafe { asm!("msr cntp_cval_el0, {}", in(reg) v) };
}

/// See the AArch32 sibling above.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
fn write_cntp_ctl(v: u32) {
    let v = v as u64;
    unsafe { asm!("msr cntp_ctl_el0, {}", in(reg) v) };
}

/// See the AArch32 sibling above.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
fn read_cntp_ctl() -> u32 {
    let ctl: u64;
    unsafe { asm!("mrs {}, cntp_ctl_el0", out(reg) ctl) };
    ctl as u32
}
