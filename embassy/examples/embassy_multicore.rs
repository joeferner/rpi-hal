#![no_std]
#![no_main]

// One executor per core: cores 1-3 each run their own `Executor` alongside
// core 0's, with the tasks on all four driven by the same `embassy-time`
// deadlines.
//
// The claim under test is that a *single* System Timer compare can serve
// tasks belonging to any core's executor. Compare 1's interrupt is
// delivered to core 0 alone -- the legacy interrupt controller routes
// GPU-side sources to one core, and nothing here re-routes it -- so a
// deadline for a task on core 3 is serviced along this path: core 0 takes
// the interrupt, drains the driver's timer queue, finds core 3's waker and
// enqueues that task onto core 3's run queue; the pender's `sev` wakes
// every core out of `wfe`; core 3 polls, and the `await` returns.
//
// Cores 1-3 never enable interrupts at all (see `core_executor`), which is
// what makes that the only available explanation for the output: `sev` is
// the sole thing that can wake them, so a counter that advances is a
// counter whose core was woken by core 0's interrupt handler.
//
// What each field is evidence for:
//
// - `coreN=` climbing means core N's executor is polling and its deadlines
//   are firing. A core that never started, or started and never woke,
//   leaves its count at 0.
// - The three periods are deliberately different -- 100ms, 200ms, 500ms --
//   so after `t` seconds the counts should read about `10t`, `5t` and `2t`.
//   Three matching rates prove considerably more than three equal ones
//   would: the driver arms Compare 1 for the nearest deadline in one
//   *global* queue, so three different periods each advancing correctly is
//   what shows that queue ordering deadlines across cores, rather than one
//   core's alarm incidentally covering the rest.
//
//   Each count sits a fixed 0 or 1 behind its exact multiple rather than
//   landing on it -- `10t-1`, `5t-1`, `2t`, say. Every period here divides
//   a whole second, so each tick task comes due at the same instant as the
//   reporting task, and the offset is just whether that core's increment
//   landed before or after this core's read of it. Which side it falls on
//   differs per core and stays put. What matters is that the offset is
//   *constant*: a lost tick anywhere would make the gap grow, so a
//   difference that never changes is a statement about read ordering and
//   nothing else. `embassy_blink` sees the same thing between two tasks on
//   one executor; the only new part here is that the read and the write are
//   on different cores.
// - `id=` is each task's own `MPIDR` affinity, read inside the task on
//   whichever core polls it. This is the check that the work really is
//   distributed rather than core 0 quietly running all of it; `id=-1` means
//   the task has not been polled even once.
// - `drift` is core 0's own deadline accuracy, the same measurement
//   `embassy_blink` makes, and it should stay as flat here as it does
//   there. Three extra cores contending for the driver's
//   `critical-section` -- a real cross-core spinlock under `rpi-hal`'s
//   `multicore` feature -- is the thing that could plausibly cost core 0
//   its deadlines, and this is the number that would show it.
//
// Requires the `multicore` feature (see Cargo.toml). No wiring beyond the
// serial console.

use core::fmt::Write as _;
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use embassy_time::{Duration, Instant, Ticker};
use rpi_hal::halt;
use rpi_hal::multicore::{Cores, Stack};
use rpi_hal::{irq, lic::Lic, pac, timer::Timer, uart::Uart};
use rpi_hal_embassy::{Executor, time_driver};

/// Tick period for each secondary core's task, indexed by slot (core 1 is
/// slot 0). Deliberately unequal, and deliberately not multiples that all
/// coincide often: distinct rates are what make the counts evidence about
/// the shared queue's ordering rather than about one alarm.
const PERIODS_MS: [u64; 3] = [100, 200, 500];

/// Ticks completed per secondary core, indexed by slot. Written by the
/// core that owns the task, read by core 0's reporting task.
static TICKS: [AtomicU32; 3] = [const { AtomicU32::new(0) }; 3];

/// The core id each tick task observed for itself on its first poll, or
/// -1 while it has never been polled.
static OBSERVED_CORE: [AtomicI32; 3] = [const { AtomicI32::new(-1) }; 3];

/// The calling core's id (0-3), from `MPIDR`'s Aff0 field.
///
/// `rpi-hal` reads this register internally for its own per-core work but
/// does not expose it, so an example that wants to name the core it is
/// running on reads it here.
#[cfg(target_arch = "arm")]
fn core_id() -> i32 {
    let mpidr: u32;
    // SAFETY: a read of a system register with no side effects, valid and
    // unprivileged-safe at the privilege level this code runs at.
    unsafe { core::arch::asm!("mrc p15, 0, {}, c0, c0, 5", out(reg) mpidr) };
    (mpidr & 3) as i32
}

/// See the AArch32 sibling above.
#[cfg(target_arch = "aarch64")]
fn core_id() -> i32 {
    let mpidr: u64;
    // SAFETY: as above.
    unsafe { core::arch::asm!("mrs {}, mpidr_el1", out(reg) mpidr) };
    (mpidr & 3) as i32
}

/// Widens a borrow of a local to `'static`.
///
/// Sound only in a function that never returns, where the value cannot go
/// out of scope while the borrow lives -- which is every caller below.
unsafe fn make_static<T>(t: &mut T) -> &'static mut T {
    unsafe { core::mem::transmute(t) }
}

/// A stack per secondary core. Each core's `Executor` is a local in its
/// entry function and so lives here rather than in `.bss`; `raw::Executor`
/// is small and the task futures sit in the pool's own statics, so most of
/// this is headroom.
static mut CORE1_STACK: Stack<0x4000> = Stack::new();
static mut CORE2_STACK: Stack<0x4000> = Stack::new();
static mut CORE3_STACK: Stack<0x4000> = Stack::new();

/// Counts deadlines on a secondary core, one task per core.
///
/// The pool holds three because three cores spawn from it. They can do that
/// concurrently: claiming a slot is a `compare_exchange` on the task's own
/// state word, so two cores reaching for the pool at once come away with
/// different slots rather than the same one.
#[embassy_executor::task(pool_size = 3)]
async fn tick(slot: usize) {
    // Recorded on the first poll, which is the first moment this task is
    // running on the core that owns it.
    OBSERVED_CORE[slot].store(core_id(), Ordering::Relaxed);

    let mut ticker = Ticker::every(Duration::from_millis(PERIODS_MS[slot]));
    loop {
        ticker.next().await;
        TICKS[slot].fetch_add(1, Ordering::Relaxed);
    }
}

/// Runs on core 0: reports every core's progress once a second.
#[embassy_executor::task]
async fn report(mut uart: Uart) {
    let start = Instant::now();
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut secs: u64 = 0;

    loop {
        ticker.next().await;
        secs += 1;

        // Cumulative rather than per-interval, for the reason
        // `embassy_blink` spells out: measuring against a single start
        // instant leaves a systematic lag nowhere to hide.
        let elapsed = start.elapsed().as_micros() as i64;
        let drift = elapsed - (secs * 1_000_000) as i64;

        let _ = write!(uart, "t={secs}s");
        for slot in 0..3 {
            let _ = write!(
                uart,
                " core{}={}(id={})",
                slot + 1,
                TICKS[slot].load(Ordering::Relaxed),
                OBSERVED_CORE[slot].load(Ordering::Relaxed),
            );
        }
        let _ = writeln!(uart, " drift={drift}us");
    }
}

/// Builds this core's executor and runs its tick task on it, forever.
///
/// The executor is created here, on the core that will poll it, rather than
/// handed over from core 0. That is what `Executor`'s `!Send` asks for:
/// `run` polls from a single context, and the `wfe`/`sev` pairing it idles
/// on is per-core.
///
/// Interrupts are left masked, which is the state every core boots in --
/// only core 0 calls `irq::enable_irq`. These three need nothing else,
/// since `wfe` returns on an event regardless of the interrupt mask, and
/// the pender's `sev` is a broadcast.
fn core_executor(slot: usize) -> ! {
    // SAFETY: this function never returns, so `executor` outlives the
    // widened borrow -- the same argument `kmain` makes for its own.
    let mut executor = Executor::new();
    let executor = unsafe { make_static(&mut executor) };

    executor.run(|spawner| {
        spawner.spawn(tick(slot).unwrap());
    })
}

extern "C" fn core1_main() -> ! {
    core_executor(0)
}

extern "C" fn core2_main() -> ! {
    core_executor(1)
}

extern "C" fn core3_main() -> ! {
    core_executor(2)
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC on core {}: {info}", core_id());
    halt();
}

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "embassy per-core executors");

    // The driver and this core's interrupt mask come first, before any
    // secondary core exists: they start awaiting deadlines the instant they
    // are spawned, and core 0 is the only core that can service Compare 1
    // for them.
    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);
    irq::enable_irq();

    // SAFETY: `Cores::steal` is called once here and nowhere else, each
    // stack is a dedicated static passed to exactly one `spawn`, and each
    // entry function never returns. The state those entries share with
    // this core is the counters above, the task pool, and the driver's
    // timer queue -- atomics, or guarded by the cross-core
    // `critical-section` that this example's `multicore` feature selects.
    unsafe {
        // Bound as raw pointers first. An `&mut` to a `static mut` cannot be
        // taken directly in edition 2024, and naming the pointers is also
        // what makes the aliasing claim legible: each is dereferenced
        // exactly once, right here, and the resulting borrow goes straight
        // into the spawn that consumes it.
        let stack1 = &raw mut CORE1_STACK;
        let stack2 = &raw mut CORE2_STACK;
        let stack3 = &raw mut CORE3_STACK;

        let cores = Cores::steal();
        cores.core1.spawn(&mut *stack1, core1_main);
        cores.core2.spawn(&mut *stack2, core2_main);
        cores.core3.spawn(&mut *stack3, core3_main);
    }

    // SAFETY: as in `core_executor` -- `kmain` never returns.
    let mut executor = Executor::new();
    let executor = unsafe { make_static(&mut executor) };

    executor.run(|spawner| {
        spawner.spawn(report(uart).unwrap());
    });
}

/// Runs on core 0 only: it is the only core with interrupts unmasked, and
/// the legacy controller routes Compare 1 to it alone.
#[unsafe(no_mangle)]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    if lic.is_timer1_pending() {
        // Acknowledges the match itself; nothing else here may clear it.
        time_driver::on_timer_irq();
    }
}
