#![no_std]
#![no_main]

// The time driver with an *empty* timer queue: what happens when no task
// is waiting on a deadline, and a Compare 1 match arrives anyway.
//
// Every other example keeps at least one deadline outstanding at all
// times, which hides one state of the driver. `Queue::next_expiration`
// returns `u64::MAX` when nothing un-expired remains, and
// `Timer::set_compare1` clamps that to ~35.8 minutes out — so with an
// idle queue the driver is sitting on an alarm no task asked for, and
// `on_timer_irq`'s claim that a spurious call "simply re-arms" is what
// keeps that harmless.
//
// Waiting out a real 35.8-minute clamp to test this is impractical, and
// shrinking the clamp means editing rpi-hal. Neither is necessary: with
// the queue empty, whatever sits in Compare 1 is owed to nobody, so this
// example overwrites it with a 5ms deadline and lets the driver take the
// match it never armed. Twenty of those land in a tenth of a second.
//
// The phases, and what each one is evidence for:
//
// 1. Three ordinary 200ms sleeps. Baseline — the deadline path works
//    before anything is poked, so a phase 4 failure means phase 3 broke
//    it rather than it never having worked.
// 2. The task stops using `embassy-time` entirely and parks on the UART.
//    An awaited *peripheral* keeps the executor alive without putting
//    anything in the timer queue, which is the state under test; a task
//    that had merely finished would leave the executor with nothing to
//    wake for at all.
// 3. Twenty injected matches, serviced with an empty queue. `spurious=20`
//    then *stopping* is the evidence: the re-arm neither wedges (the
//    `while !set_compare1(..)` retry in `rearm` cannot spin here, since
//    `u64::MAX` is never in the past) nor turns one spurious match into a
//    storm of them. `clamp=` reads back the alarm the driver armed on its
//    own after the last injection: ~0x80000000 ahead of the counter,
//    which is the clamp arithmetic and its `as u32` truncation stated as
//    a number rather than assumed.
// 4. A fresh 500ms deadline, after all of that. This is the payoff — the
//    spurious matches left the queue and the compare in a state where a
//    *new* deadline is still honoured to within a few microseconds.
//
// The keypress is part of the test, not just a prompt. It arrives long
// after the injections finish, so the core has been asleep in `wfe` on an
// alarm 35.8 minutes out; the console waking at all is what shows the
// idle driver parked the core rather than losing it.
//
// No extra wiring — the serial console is all this needs.

use core::fmt::Write as _;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_time::{Duration, Instant, Timer as EmbassyTimer};
use embedded_io_async::Read;
use rpi_hal::halt;
use rpi_hal::pac::SYSTMR;
use rpi_hal::{irq, lic::Lic, pac, timer::Timer, uart::Uart};
use rpi_hal_embassy::{Executor, time_driver};

/// How many matches to inject with the queue empty. Enough that a driver
/// which mishandled one would have to mishandle it repeatedly, few enough
/// that the whole phase is over before a key can be pressed.
const SPURIOUS_TARGET: u32 = 20;

/// Gap between injected matches. Comfortably longer than the handler
/// itself, so each one is a separate interrupt rather than a match that
/// had already passed by the time it was armed.
const INJECT_INTERVAL_US: u64 = 5_000;

/// What the clamp should put in Compare 1, mirroring rpi-hal's
/// `MAX_COMPARE1_DELTA_US`. Not imported because it is private there —
/// this example asserts the documented ~35.8 minutes, which is the part
/// the driver's callers actually depend on.
const EXPECTED_CLAMP_US: u64 = 1 << 31;

/// Whether the handler should keep injecting matches. Set once the task
/// has stopped using `embassy-time`, so the handler never poisons a
/// deadline a task is genuinely waiting for.
static INJECT: AtomicBool = AtomicBool::new(false);

/// Matches serviced during the injection phase.
static SPURIOUS: AtomicU32 = AtomicU32::new(0);

/// Compare 1 as the driver left it after the final injected match — the
/// alarm it armed for an empty queue.
static CLAMP_C1: AtomicU32 = AtomicU32::new(0);

/// The counter's low word, sampled just after [`CLAMP_C1`], so the two
/// can be subtracted to recover the clamp distance.
static CLAMP_NOW: AtomicU32 = AtomicU32::new(0);

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[embassy_executor::task]
async fn probe(mut uart: Uart) {
    // Blocking `writeln!` rather than the async writer the UART examples
    // use: this is the only task, so a few milliseconds spent in the
    // transmit FIFO steals time from nothing. Keeping the writes
    // synchronous also keeps the timer queue provably empty during phase
    // 3 — an async write parks on the UART, not on a deadline, but not
    // having to reason about that is worth more here than the throughput.

    // Phase 1: ordinary deadlines, before anything is interfered with.
    for i in 1..=3 {
        let start = Instant::now();
        EmbassyTimer::after(Duration::from_millis(200)).await;
        let _ = writeln!(
            uart,
            "phase 1: sleep {i}/3 took {}us",
            start.elapsed().as_micros()
        );
    }

    // Phase 2/3: hand the compare to the interrupt handler and stop
    // scheduling deadlines. The `await` below is on the UART, so from
    // here until a key is pressed the timer queue holds nothing.
    let _ = writeln!(
        uart,
        "phase 3: injecting {SPURIOUS_TARGET} matches every {INJECT_INTERVAL_US}us with an empty queue"
    );
    let _ = writeln!(uart, "         press a key once they have landed");

    let timer = Timer::new(unsafe { SYSTMR::steal() });
    INJECT.store(true, Ordering::Relaxed);
    while !timer.set_compare1(timer.now_micros() + INJECT_INTERVAL_US) {}

    let mut buf = [0u8; 32];
    let _ = uart.read(&mut buf).await;

    // A key pressed before the injections had finished would overlap
    // phases 3 and 4, and the number phase 4 reports is only meaningful
    // with nothing interfering. Spun on rather than awaited: the handler
    // is what clears this, and an `await` on a deadline would put an
    // entry back in the queue phase 3 is still meant to find empty.
    while INJECT.load(Ordering::Relaxed) {
        core::hint::spin_loop();
    }

    // Phase 3 results. `spurious` short of the target here cannot happen
    // -- the wait above ended when the handler stopped injecting, which
    // it only does on reaching the target -- so it is printed as the
    // count that the clamp reading below belongs to.
    let spurious = SPURIOUS.load(Ordering::Relaxed);
    let _ = writeln!(uart, "phase 3: spurious={spurious}");

    // Compare 1 minus the counter, in wrapping 32-bit arithmetic, which
    // is the only arithmetic the hardware compare does. `skew` is how far
    // short of the clamp that lands: the time between the driver arming
    // it and the handler reading it back, which is a few register reads
    // -- under the counter's 1us resolution, so expect 0 or -1 rather
    // than exactly 0.
    let delta = CLAMP_C1
        .load(Ordering::Relaxed)
        .wrapping_sub(CLAMP_NOW.load(Ordering::Relaxed));
    let skew = delta as i64 - EXPECTED_CLAMP_US as i64;
    let _ = writeln!(
        uart,
        "phase 3: clamp={delta}us (expected {EXPECTED_CLAMP_US}, skew {skew}us)"
    );

    // Phase 4: the driver has to still be usable.
    let start = Instant::now();
    EmbassyTimer::after(Duration::from_millis(500)).await;
    let error = start.elapsed().as_micros() as i64 - 500_000;
    let _ = writeln!(
        uart,
        "phase 4: 500ms deadline after the storm, error={error}us"
    );

    let _ = writeln!(uart, "done");
}

// `unsafe(no_mangle)` rather than the bare `no_mangle` rpi-hal's own
// examples use: this crate is edition 2024, which requires the wrapper.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "embassy: time driver with an idle queue");

    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);

    // The UART is a wake source here for the same reason the timer is:
    // the task parks on it, and phase 3 is only a test of the *idle*
    // queue if the thing keeping the executor alive is not a deadline.
    lic.enable_uart_irq();
    irq::enable_irq();

    // The executor must outlive `run`. Widening a local's lifetime is
    // sound only because `kmain` never returns -- the same trick
    // `#[embassy_executor::main]` performs, and the reason that macro
    // requires a diverging entry point.
    unsafe fn make_static<T>(t: &mut T) -> &'static mut T {
        unsafe { core::mem::transmute(t) }
    }
    let mut executor = Executor::new();
    let executor = unsafe { make_static(&mut executor) };

    executor.run(|spawner| {
        spawner.spawn(probe(uart).unwrap());
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    if lic.is_timer1_pending() {
        // Acknowledges the match and re-arms, whether or not anything was
        // due -- which during the injection phase means arming the clamp
        // for an empty queue, the path this example exists to run.
        time_driver::on_timer_irq();

        if INJECT.load(Ordering::Relaxed) {
            // Relaxed throughout: the handler and the task never run
            // concurrently on one core -- the handler interrupts the
            // task and returns before it resumes -- so there is no
            // ordering for a stronger operation to establish.
            let n = SPURIOUS.fetch_add(1, Ordering::Relaxed) + 1;
            let timer = Timer::new(unsafe { SYSTMR::steal() });

            if n < SPURIOUS_TARGET {
                // Retried rather than ignored: `false` would mean the
                // interval elapsed inside this handler, leaving nothing
                // armed and stalling the phase silently rather than
                // failing it.
                while !timer.set_compare1(timer.now_micros() + INJECT_INTERVAL_US) {}
            } else {
                // Leave the driver's own arm in place and record it.
                // Compare 1 first: reading the counter afterwards makes
                // the difference between them an underestimate of the
                // clamp by the time the two reads take, never an
                // overestimate that could hide a short arm.
                CLAMP_C1.store(
                    unsafe { SYSTMR::steal() }.c1().read().bits(),
                    Ordering::Relaxed,
                );
                CLAMP_NOW.store(timer.now_micros() as u32, Ordering::Relaxed);
                INJECT.store(false, Ordering::Relaxed);
            }
        }
    }

    if lic.is_uart_pending() {
        rpi_hal::uart::on_irq();
    }
}
