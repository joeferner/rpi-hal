#![no_std]
#![no_main]

// Interrupt-driven SD transfers: the async half of `sd.rs`, with no
// executor crate involved.
//
// The blocking driver spins on `INTERRUPT` for every handshake, so a card
// access costs the CPU the whole of it -- including the milliseconds a
// write spends waiting for the card to program an erase block, which is
// the largest single block of wasted time in either of the applications
// built on this crate. The async methods park on the controller's
// interrupt instead. This checks that they do, that they return the same
// bytes, and that a cancelled transfer leaves the card usable.
//
// The measurement is what makes it worth running. `block_on` below parks
// the core in `wfe` between polls and adds up how long it stays there, so
// each transfer reports the share of its own duration during which the
// core had nothing to do -- which under a real executor is the share that
// would go to other tasks. The blocking transfers it is measured against
// report no idle share at all, because there isn't any: their polling
// loop is the CPU. Expect the async and blocking timings for the same
// transfer to be about equal; the point is not that it is faster, it is
// that the time is given back.
//
// The checks, in order:
//
//  1. **A single-block read agrees with the blocking one.** The cheapest
//     way to find out whether the interrupt reaches the core at all: if
//     it doesn't, this hangs at the first await rather than printing
//     anything wrong, which is what a missing `__irq_handler` or an
//     unrouted `enable_emmc_irq` looks like.
//  2. **A multi-block read agrees**, over `RUN_BLOCKS` blocks -- several
//     `READ_RDY` handshakes rather than one, so the per-block await is
//     exercised too.
//  3. **The DMA read agrees**, with the whole data phase moved onto a DMA
//     channel and awaited as one wakeup.
//  4. **A cancelled transfer leaves the card usable.** A multi-block read
//     is polled once (which issues the command and parks it) and then
//     dropped, abandoning the card mid-stream; check 1 is then repeated
//     and must still pass. A broken cleanup shows up on that *next*
//     transfer, not on the cancelled one, which is what makes it worth
//     testing deliberately rather than waiting to meet it.
//  5. **A write round trip**, opt-in and self-restoring (see
//     `RUN_WRITE_TEST`), timed against the blocking write of the same
//     data. This is the check the whole feature is for: the async write's
//     idle share should be most of its duration.

use core::fmt::Write;
use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use rpi_hal::dma::Dma;
use rpi_hal::sd::{Block, Sd};
use rpi_hal::{halt, irq, lic::Lic, mailbox::Mailbox, pac, sd, timer::Timer, uart::Uart};

/// Number of consecutive blocks the multi-block checks move. Eight
/// 512-byte blocks (4 KiB) spans several `READ_RDY` handshakes and a DMA
/// run of real length while staying small.
const RUN_BLOCKS: usize = 8;

/// Whether to run the (opt-in) write round trip. It restores what it
/// found, so a completed run is non-destructive, but a bug in the write
/// path could leave `WRITE_TEST_BLOCK`..`+RUN_BLOCKS` holding inverted
/// data -- the same trade `sd_multi_block.rs` documents at more length.
const RUN_WRITE_TEST: bool = true;

/// First block the write round trip touches, kept high (512 MiB in) to
/// steer clear of a card's partition table and filesystem. Only consulted
/// when `RUN_WRITE_TEST`.
const WRITE_TEST_BLOCK: u32 = 0x0010_0000;

/// A run of blocks aligned to a cache line, so the DMA cache maintenance
/// on it never spills onto neighbouring data (each block is already a
/// whole number of 64-byte lines; this fixes the start too).
#[repr(C, align(64))]
struct Blocks([Block; RUN_BLOCKS]);

/// Read by the blocking path, and the reference every async read is
/// compared against.
static mut REFERENCE: Blocks = Blocks([[0; 512]; RUN_BLOCKS]);
/// Filled by each async read in turn.
static mut SCRATCH: Blocks = Blocks([[0; 512]; RUN_BLOCKS]);

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Services the interrupt every await here depends on.
///
/// Mandatory, and silently fatal to omit: `rpi-hal` provides only a weak
/// no-op `__irq_handler`, so without this the first transfer parks, the
/// controller keeps asserting its line, and the core either re-enters the
/// handler forever or never wakes. On the console both look like a hang
/// at the first await rather than like an error.
#[no_mangle]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    if lic.is_emmc_pending() {
        sd::on_irq();
    }
}

/// How much of a transfer the core spent with nothing to do.
struct Idle {
    /// Times the core came out of `wfe` -- one per interrupt, plus any
    /// spurious wake-ups the instruction is allowed to take.
    wakeups: u32,
    /// Microseconds spent parked, summed across those.
    parked_us: u64,
}

impl Idle {
    /// The parked time as a percentage of `elapsed_us` — what an executor
    /// would have had for other work.
    fn share_of(&self, elapsed_us: u64) -> u64 {
        (self.parked_us * 100).checked_div(elapsed_us).unwrap_or(0)
    }
}

/// Wakes a core parked in `wfe`.
///
/// `dsb ish` before `sev` is ARM's prescribed order for signalling other
/// observers; here it also orders the waker's own stores before the
/// wake-up is broadcast.
fn signal_event() {
    // SAFETY: neither instruction has operands or touches memory.
    unsafe { core::arch::asm!("dsb ish", "sev", options(nomem, nostack)) };
}

/// A [`Waker`] whose only job is to make a `wfe` return.
///
/// There is nothing to store per-waker — the loop below re-polls its one
/// future on any wake-up — so the data pointer is null and the vtable's
/// clone/drop are no-ops.
fn event_waker() -> Waker {
    fn wake(_: *const ()) {
        signal_event();
    }
    fn clone(data: *const ()) -> RawWaker {
        RawWaker::new(data, &VTABLE)
    }
    fn drop(_: *const ()) {}

    static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake, drop);

    // SAFETY: the vtable's functions are valid for the null data pointer
    // they are given -- none of them dereferences it.
    unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
}

/// Drives one future to completion, parking the core between polls, and
/// reports how long it stayed parked.
///
/// No lost wake-up race, for the reason `rpi-hal-embassy`'s executor
/// relies on: `sev` sets the core's event register whether or not
/// anything is waiting on it, so a wake that lands after the last poll
/// and before the `wfe` simply makes that `wfe` return immediately.
///
/// A real executor would run other tasks where this parks. Measuring the
/// parked time is the closest a single-future program can get to
/// measuring what those tasks would receive.
fn block_on<F: Future>(future: F, timer: &Timer) -> (F::Output, Idle) {
    let mut future = pin!(future);
    let waker = event_waker();
    let mut context = Context::from_waker(&waker);
    let mut idle = Idle {
        wakeups: 0,
        parked_us: 0,
    };

    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
            return (output, idle);
        }
        let entered = timer.now_micros();
        // SAFETY: `wfe` has no operands; at worst it returns immediately.
        unsafe { core::arch::asm!("wfe", options(nomem, nostack)) };
        idle.parked_us += timer.now_micros() - entered;
        idle.wakeups += 1;
    }
}

/// Polls a transfer once — enough to issue the command and park it — and
/// then drops it, which is what a `with_timeout` expiring mid-transfer
/// does. The abort is the drop's own work, so there is nothing here to
/// wait for.
fn start_then_cancel<F: Future>(future: F) {
    let mut future = pin!(future);
    let waker = event_waker();
    let mut context = Context::from_waker(&waker);
    let _ = future.as_mut().poll(&mut context);
}

/// Runs `transfer` and reports its outcome, duration and idle share on one
/// line, returning whether it succeeded.
fn timed<F>(uart: &mut Uart, timer: &Timer, label: &str, transfer: F) -> bool
where
    F: Future<Output = Result<(), sd::Error>>,
{
    let started = timer.now_micros();
    let (result, idle) = block_on(transfer, timer);
    let elapsed = timer.now_micros() - started;

    match result {
        Ok(()) => {
            let _ = writeln!(
                uart,
                "{label}: ok in {elapsed}us, idle {}us ({}%) over {} wakeup(s)",
                idle.parked_us,
                idle.share_of(elapsed),
                idle.wakeups
            );
            true
        }
        Err(e) => {
            let _ = writeln!(uart, "{label}: FAILED after {elapsed}us: {e:?}");
            false
        }
    }
}

/// Compares `blocks` of `scratch` against the same prefix of `reference`,
/// reporting a pass or the number of differing bytes.
fn compare(uart: &mut Uart, label: &str, scratch: &Blocks, reference: &Blocks, blocks: usize) {
    let (read, expected) = (
        scratch.0[..blocks].as_flattened(),
        reference.0[..blocks].as_flattened(),
    );
    let differing = read.iter().zip(expected).filter(|(a, b)| a != b).count();
    if differing == 0 {
        let _ = writeln!(
            uart,
            "{label}: PASS, {blocks} block(s) match the blocking read"
        );
    } else {
        let _ = writeln!(uart, "{label}: FAIL, {differing} byte(s) differ");
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    let lic = Lic::new(peripherals.LIC);

    let _ = writeln!(uart, "\r\ninterrupt-driven SD");

    let mut sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(uart, "SD init failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "card ready ({}, {}-bit bus)",
        if sd.high_capacity() {
            "SDHC/SDXC"
        } else {
            "SDSC"
        },
        if sd.four_bit_bus() { 4 } else { 1 }
    );

    // Two of the three gates an interrupt has to pass. The third is the
    // controller's own `IRPT_EN`, which each transfer opens as it parks
    // and closes again on its way out -- so unlike every other driver
    // here, there is nothing peripheral-side to set up.
    lic.enable_emmc_irq();
    irq::enable_irq();

    // A full DMA channel (0-6) for check 3. On a bare-metal board that has
    // taken over the machine any channel works; see the DMA driver docs on
    // the firmware's channel mask if the firmware is left running.
    let mut dma = Dma::new();
    let mut channel = dma.channel(5).expect("channel 5 available");

    // SAFETY: single-threaded `kmain`; these statics are touched only
    // here and, through the references taken now, by the calls below.
    let reference = unsafe { &mut *core::ptr::addr_of_mut!(REFERENCE) };
    let scratch = unsafe { &mut *core::ptr::addr_of_mut!(SCRATCH) };

    // The reference every read is checked against, taken with the
    // blocking path so a fault in the async one cannot hide by being
    // wrong the same way twice.
    let started = timer.now_micros();
    if let Err(e) = sd.read_blocks(0, &mut reference.0, &timer) {
        let _ = writeln!(uart, "blocking reference read failed: {e:?}");
        halt();
    }
    let blocking_read_us = timer.now_micros() - started;
    let signature = &reference.0[0][510..];
    let _ = writeln!(
        uart,
        "blocking read of {RUN_BLOCKS} blocks: {blocking_read_us}us, no idle time; \
         block 0 signature {:02x} {:02x}{}",
        signature[0],
        signature[1],
        if signature == [0x55, 0xaa] {
            " (0x55AA)"
        } else {
            " (unpartitioned card, or a blank region)"
        }
    );

    // 1. Single block.
    scratch.0[0] = [0; 512];
    if timed(&mut uart, &timer, "async read_block", async {
        sd.read_block_async(0, &mut scratch.0[0], &timer).await
    }) {
        compare(&mut uart, "async read_block", scratch, reference, 1);
    }

    // 2. Multi-block, one await per block.
    scratch.0 = [[0; 512]; RUN_BLOCKS];
    if timed(&mut uart, &timer, "async read_blocks", async {
        sd.read_blocks_async(0, &mut scratch.0, &timer).await
    }) {
        compare(
            &mut uart,
            "async read_blocks",
            scratch,
            reference,
            RUN_BLOCKS,
        );
    }

    // 3. Multi-block over DMA, one await for the lot.
    scratch.0 = [[0; 512]; RUN_BLOCKS];
    if timed(&mut uart, &timer, "async read_blocks_dma", async {
        sd.read_blocks_dma_async(0, &mut scratch.0, &mut channel, &timer)
            .await
    }) {
        compare(
            &mut uart,
            "async read_blocks_dma",
            scratch,
            reference,
            RUN_BLOCKS,
        );
    }

    // 4. Cancel one part-way, then use the card again straight afterwards.
    // The second half is the real assertion.
    let _ = writeln!(uart, "cancelling a multi-block read after one poll...");
    start_then_cancel(sd.read_blocks_async(0, &mut scratch.0, &timer));
    scratch.0[0] = [0; 512];
    if timed(&mut uart, &timer, "read after cancel", async {
        sd.read_block_async(0, &mut scratch.0[0], &timer).await
    }) {
        compare(&mut uart, "read after cancel", scratch, reference, 1);
    }

    if RUN_WRITE_TEST {
        write_round_trip(&mut sd, &timer, &mut uart, reference, scratch);
    }

    let _ = writeln!(uart, "done -- halting");
    halt();
}

/// Discriminating, self-restoring write check, and the measurement the
/// feature exists for.
///
/// The card's real contents are read into `reference` first and written
/// back at the end, with a bit-inverted copy in between: reading back the
/// *inverted* data is what tells a write that landed from one that
/// silently did nothing, since a no-op would read back as the original.
/// The inverting write goes out through the blocking path and the restore
/// through the async one, so both are proven to change the medium and the
/// two can be timed against each other on the same data.
///
/// The restore is attempted whatever happened above, so the region is left
/// as it was found whenever we got far enough to modify it.
fn write_round_trip(
    sd: &mut Sd,
    timer: &Timer,
    uart: &mut Uart,
    reference: &mut Blocks,
    scratch: &mut Blocks,
) {
    if let Err(e) = sd.read_blocks(WRITE_TEST_BLOCK, &mut reference.0, timer) {
        let _ = writeln!(
            uart,
            "write test: initial read failed: {e:?} (card untouched)"
        );
        return;
    }
    for (dst, src) in scratch
        .0
        .as_flattened_mut()
        .iter_mut()
        .zip(reference.0.as_flattened())
    {
        *dst = !*src;
    }

    // The blocking write, for the comparison. Every microsecond of this
    // one belongs to the polling loop.
    let started = timer.now_micros();
    let blocking = sd.write_blocks(WRITE_TEST_BLOCK, &scratch.0, timer);
    let blocking_us = timer.now_micros() - started;
    match blocking {
        Ok(()) => {
            let _ = writeln!(
                uart,
                "blocking write_blocks: ok in {blocking_us}us, no idle time"
            );
        }
        Err(e) => {
            let _ = writeln!(
                uart,
                "blocking write_blocks: FAILED after {blocking_us}us: {e:?}"
            );
        }
    }

    // Did the inverted data actually land? Compared against `!reference`
    // rather than against `scratch`, which the read overwrites.
    match sd.read_blocks(WRITE_TEST_BLOCK, &mut scratch.0, timer) {
        Ok(()) => {
            let landed = scratch
                .0
                .as_flattened()
                .iter()
                .zip(reference.0.as_flattened())
                .all(|(read, original)| *read == !*original);
            let _ = writeln!(
                uart,
                "blocking write verify: {}",
                if landed { "PASS" } else { "FAIL, unchanged" }
            );
        }
        Err(e) => {
            let _ = writeln!(uart, "blocking write verify: read failed: {e:?}");
        }
    }

    // The async write, restoring the original — and the number this whole
    // example is about. The card's programming time is the bulk of it, and
    // all of that should show up as idle.
    let restored = timed(uart, timer, "async write_blocks", async {
        sd.write_blocks_async(WRITE_TEST_BLOCK, &reference.0, timer)
            .await
    });

    // The medium currently holds the inverted data, so a no-op async write
    // reads back inverted rather than restored: this discriminates the
    // async write as well as confirming the card is back as it was.
    match sd.read_blocks(WRITE_TEST_BLOCK, &mut scratch.0, timer) {
        Ok(()) => {
            let matches = scratch.0.as_flattened() == reference.0.as_flattened();
            let _ = writeln!(
                uart,
                "async write verify: {}",
                if matches {
                    "PASS, original restored"
                } else if restored {
                    "FAIL, region still inverted"
                } else {
                    "FAIL -- the write itself failed, region left INVERTED"
                }
            );
        }
        Err(e) => {
            let _ = writeln!(uart, "async write verify: read failed: {e:?}");
        }
    }
}
