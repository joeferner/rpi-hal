// `critical-section` implementation backed by the CPU-level IRQ mask
// (`crate::irq`), plus a cross-core spinlock when the `multicore`
// feature is on.
//
// Without `multicore`, masking IRQ alone is enough to exclude the only
// other context (the IRQ handler) that could touch shared state
// concurrently, on this single-core-only target with no FIQ usage
// anywhere in this crate. That stops being true the moment a second
// core is actually running: masking IRQ on this core has no effect on
// another one, so `multicore` adds a real cross-core lock on top.
//
// Registration (`set_impl!`) is a side effect of this module existing
// at all — nothing here is meant to be called directly. Consumers use
// the `critical_section` crate's own public API (`critical_section::
// with`, etc.), which dispatches to this implementation at link time.

use critical_section::{set_impl, Impl, RawRestoreState};

#[cfg(feature = "multicore")]
use core::sync::atomic::{AtomicBool, Ordering};

struct CriticalSection;
set_impl!(CriticalSection);

/// Cross-core mutual exclusion, on top of the CPU-level IRQ mask below
/// — `compare_exchange_weak` needs `ldrex`/`strex`, which per
/// `mmu.rs`'s own doc comment requires the MMU/cacheable-RAM setup
/// `mmu` provides (implied by `multicore` — see `Cargo.toml`) to behave
/// correctly on this core.
#[cfg(feature = "multicore")]
static LOCK: AtomicBool = AtomicBool::new(false);

unsafe impl Impl for CriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        // Read the prior IRQ-mask state before disabling so `release`
        // can restore it correctly — critical sections must nest, and
        // an inner one must not re-enable IRQ that was already masked
        // by an outer one. The IRQ mask lives in the CPSR `I` bit on
        // AArch32 and in `PSTATE.I` (read via `DAIF`) on AArch64; in
        // both, bit 7 of the read value is the I bit, set when IRQ is
        // masked.
        #[cfg(target_arch = "arm")]
        let irq_state: u32 = {
            let cpsr: u32;
            core::arch::asm!("mrs {0}, cpsr", out(reg) cpsr);
            cpsr
        };
        #[cfg(target_arch = "aarch64")]
        let irq_state: u64 = {
            let daif: u64;
            core::arch::asm!("mrs {0}, DAIF", out(reg) daif);
            daif
        };
        let was_enabled = irq_state & (1 << 7) == 0;
        crate::irq::disable_irq();

        // Mask IRQ on this core first (above), then wait for every
        // other core to leave its own critical section — masking IRQ
        // alone only excludes this core's own IRQ handler, not another
        // core entirely.
        #[cfg(feature = "multicore")]
        while LOCK
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            core::hint::spin_loop();
        }

        was_enabled
    }

    unsafe fn release(was_enabled: RawRestoreState) {
        #[cfg(feature = "multicore")]
        LOCK.store(false, Ordering::Release);

        if was_enabled {
            crate::irq::enable_irq();
        }
    }
}
