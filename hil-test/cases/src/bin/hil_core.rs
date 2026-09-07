//! Core SoC drivers that can self-check with nothing attached but a console.
//!
//! Everything here asserts against something the board itself can compute, so
//! this runs on the smoke tier alongside `hil_smoke`. Split from that binary
//! because `hil_smoke` is the minimum a contributor runs to confirm a board
//! is alive, and keeping it short keeps it fast to read when it fails.

#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;
use core::sync::atomic::{AtomicU32, Ordering};

use embedded_alloc::LlffHeap as Heap;
use hil_cases::{hil_panic_handler, Session};
use rpi_hal::dma::Dma;
use rpi_hal::generic_timer::GenericTimer;
use rpi_hal::mailbox::{ClockId, Mailbox};
use rpi_hal::pac;
use rpi_hal::rng::Rng;
use rpi_hal::timer::Timer;

hil_panic_handler!();

/// Enough to make the allocator do real work without depending on how much
/// RAM the firmware left for the ARM.
const HEAP_SIZE: usize = 16 * 1024;

/// A fixed region rather than everything above `.bss` up to the ARM/VideoCore
/// split. This case is testing that the allocator works, not how much memory
/// the firmware handed over, and a static array keeps that assertion
/// independent of the mailbox -- which is separately under test below.
static mut HEAP_MEM: [u8; 4 * HEAP_SIZE] = [0; 4 * HEAP_SIZE];

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    // Before anything that allocates.
    unsafe {
        let base = &raw mut HEAP_MEM;
        HEAP.init(base as usize, 4 * HEAP_SIZE);
    }

    let mut session = Session::start(9);

    let peripherals = unsafe { pac::Peripherals::steal() };
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // The MMU, asserted through something only it can make work.
    //
    // `compare_exchange` compiles to ldrex/strex (or LSE), which the
    // architecture only guarantees against Normal Cacheable Shareable
    // memory. With the MMU off, every address is Device/Strongly-Ordered and
    // the exclusive monitor never succeeds — so a successful swap is
    // evidence the identity map is real and the attributes are right, not
    // merely that the feature compiled in.
    static CELL: AtomicU32 = AtomicU32::new(0xDEAD);
    let swapped = CELL
        .compare_exchange(0xDEAD, 0xBEEF, Ordering::AcqRel, Ordering::Acquire)
        .is_ok();
    session.check(
        "mmu_exclusives",
        swapped && CELL.load(Ordering::Acquire) == 0xBEEF,
        "compare_exchange failed: memory is not Normal Cacheable, so the MMU map is wrong",
    );

    // The heap. Grow a Vec past its inline capacity so this exercises
    // reallocation rather than a single lucky allocation, and read the
    // contents back so a bad copy shows up.
    let mut grown: Vec<u32> = Vec::new();
    for i in 0..(HEAP_SIZE / 4) as u32 {
        grown.push(i ^ 0x5A5A_5A5A);
    }
    let intact = grown
        .iter()
        .enumerate()
        .all(|(i, &v)| v == (i as u32) ^ 0x5A5A_5A5A);
    session.check(
        "alloc_heap",
        intact && grown.len() == HEAP_SIZE / 4,
        "a grown Vec did not read back what was written",
    );

    // The hardware RNG. Distinct values rule out a stuck register, which is
    // what a disabled or unclocked RNG reads as — and a stuck generator is
    // far worse than an absent one, because callers cannot tell.
    let mut rng = Rng::new();
    let draws = [
        rng.next_u32(),
        rng.next_u32(),
        rng.next_u32(),
        rng.next_u32(),
    ];
    let all_same = draws.iter().all(|&v| v == draws[0]);
    let any_trivial = draws.iter().all(|&v| v == 0 || v == u32::MAX);
    session.check(
        "rng_varies",
        !all_same && !any_trivial,
        "successive reads were identical or trivial; the RNG looks unclocked",
    );

    // DMA memcpy, checked byte for byte. A pattern rather than zeroes, so a
    // transfer that silently moves nothing cannot pass.
    let mut dma = Dma::new();
    let channel = dma.channel(0);
    let mut src = [0u8; 256];
    for (i, b) in src.iter_mut().enumerate() {
        *b = (i as u8) ^ 0xA5;
    }
    let mut dest = [0u8; 256];
    match channel.map(|mut c| c.memcpy(&mut dest, &src)) {
        Some(Ok(())) => session.check(
            "dma_memcpy",
            dest == src,
            "the destination does not match the source after a DMA copy",
        ),
        Some(Err(_)) => session.check("dma_memcpy", false, "the DMA transfer reported an error"),
        None => session.skip("dma_memcpy", "DMA channel 0 is not available"),
    }

    // The per-core architected timer's frequency, checked before it is used
    // to scale anything.
    //
    // `CNTFRQ` is not populated by hardware — it is a register firmware is
    // expected to program at the highest exception level, and nothing forces
    // it to. So it is checked on its own rather than folded into the
    // comparison below: a zero or nonsense frequency makes every duration
    // derived from this timer wrong, and that is a different fault from two
    // working clocks disagreeing.
    let generic = GenericTimer::new();
    let freq = generic.frequency();
    let freq_sane = (1_000_000..=1_000_000_000).contains(&freq);
    session.check_fmt(
        "generic_timer_frequency",
        freq_sane,
        format_args!("CNTFRQ reads {freq} Hz, outside 1MHz-1GHz; firmware may not have set it"),
    );

    // Diagnostics for the counter's actual rate, recorded rather than
    // asserted. `CNTFRQ` is only a claim: nothing in the architecture ties it
    // to how fast `CNTPCT` really advances, and on this hardware the rate
    // comes from the ARM-local timer control and prescaler registers, which
    // firmware is expected to set. Measuring the rate against the System
    // Timer and dumping those two registers alongside it says whether a
    // disagreement is a wrong claim or a wrong clock.
    {
        // SAFETY: ARM-local peripherals, device-mapped by rpi-hal's MMU
        // bring-up — the same block `route_irq` writes to. Reads only.
        let (control, prescaler) = unsafe {
            (
                core::ptr::read_volatile(0x4000_0000 as *const u32),
                core::ptr::read_volatile(0x4000_0008 as *const u32),
            )
        };
        let st_start = timer.now_micros();
        let gt_start = generic.now();
        timer.delay_us(20_000);
        let ticks = generic.now() - gt_start;
        let micros = timer.now_micros().wrapping_sub(st_start).max(1);
        let measured_hz = (ticks as u128 * 1_000_000 / micros as u128) as u64;

        session.note("cntfrq_hz", format_args!("{freq}"));
        session.note("cntpct_measured_hz", format_args!("{measured_hz}"));
        session.note("local_timer_control", format_args!("{control:#010x}"));
        session.note("local_timer_prescaler", format_args!("{prescaler:#010x}"));
    }

    // Cross-checked against the System Timer. Two independent clocks
    // agreeing is a far stronger claim than either alone: a single timer can
    // only be checked against a delay driven by itself, which is circular.
    if freq_sane {
        let gt_start = generic.now();
        let st_start = timer.now_micros();
        timer.delay_us(50_000);
        let gt_ticks = generic.now() - gt_start;
        let gt_elapsed_us = (gt_ticks as u128 * 1_000_000 / freq as u128) as u64;
        let st_elapsed_us = timer.now_micros().wrapping_sub(st_start);
        let skew = gt_elapsed_us.abs_diff(st_elapsed_us);
        session.check_fmt(
            "generic_timer_agrees_with_system_timer",
            skew < 5_000,
            format_args!(
                "over one 50ms delay the generic timer measured {gt_elapsed_us}us \
                 ({gt_ticks} ticks at {freq}Hz) and the System Timer {st_elapsed_us}us, \
                 a skew of {skew}us"
            ),
        );
    } else {
        session.skip(
            "generic_timer_agrees_with_system_timer",
            "CNTFRQ is not usable, so ticks cannot be converted to a duration",
        );
    }

    // Clock rates. The core clock has to be a plausible non-zero rate, and
    // no greater than the maximum the firmware itself reports — an ordering
    // the firmware should never violate, so a violation means the property
    // interface is returning something other than what was asked for.
    match (
        mailbox.clock_rate_hz(ClockId::Arm),
        mailbox.max_clock_rate_hz(ClockId::Arm),
    ) {
        (Ok(rate), Ok(max)) => session.check(
            "mailbox_clock_rate",
            rate > 0 && max > 0 && rate <= max,
            "the ARM clock rate is zero, or above the reported maximum",
        ),
        _ => session.check("mailbox_clock_rate", false, "a clock rate query failed"),
    }

    // The ARM/VideoCore split. Both regions must be non-empty and must not
    // overlap; an overlap would mean the driver is misreading the response,
    // and writing into VideoCore's half corrupts the firmware underneath.
    match (mailbox.arm_memory(), mailbox.vc_memory()) {
        (Ok(arm), Ok(vc)) => {
            let disjoint = arm.base_address + arm.size_bytes <= vc.base_address
                || vc.base_address + vc.size_bytes <= arm.base_address;
            session.check(
                "mailbox_memory_split",
                arm.size_bytes > 0 && vc.size_bytes > 0 && disjoint,
                "the ARM and VideoCore regions are empty or overlap",
            );
        }
        _ => session.check("mailbox_memory_split", false, "a memory split query failed"),
    }

    // The SoC temperature. Bounded rather than exact: this is checking the
    // sensor is wired and scaled, not calibrating it. Below freezing or
    // above the throttle point means the units are wrong.
    match mailbox.temperature_millicelsius() {
        Ok(milli) => session.check(
            "mailbox_temperature",
            (1_000..=90_000).contains(&milli),
            "the temperature is outside 1-90C, so the scaling looks wrong",
        ),
        Err(_) => session.check("mailbox_temperature", false, "the temperature query failed"),
    }

    session.finish()
}
