#![no_std]
#![no_main]

// What the SD driver does when there is no card in the slot.
//
// No Pi has a card-detect line to read -- GPIO47, which is sometimes
// claimed to be one, is the ACT LED on a Pi 1/2, the PMIC's I2C data line
// on a Pi 3, and part of the Ethernet PHY's RGMII interface on a Pi 4;
// no board's device tree gives its SD host a `cd-gpios`, and this SoC's
// Arasan `STATUS` register doesn't implement the SDHCI present-state bits
// either. So presence is something the driver discovers by asking, not
// something it senses, and this example shows exactly what the asking
// looks like from the outside.
//
// It runs `Sd::init` on demand, once per keypress, reporting how long it
// took, which `sd::Error` came back, and -- for the two variants that
// carry controller state -- what the `INTERRUPT` and `STATUS` registers
// held, decoded bit by bit. Run it, pull the card out, press a key; put
// the card back, press a key again. The board keeps running throughout:
// the kernel is entirely in RAM by the time this starts, so the card can
// come and go without the program noticing except where it asks.
//
// An empty slot comes back as `Error::NoCard`, reached by `CMD8`
// (`SEND_IF_COND`, the first command in the identification sequence that
// expects a response at all -- `CMD0` before it expects none, so it
// cannot tell the difference) timing out, and a `CMD55` sent afterwards
// timing out too. Both are needed: `CMD8` arrived with SD 2.0, so a v1.x
// card doesn't answer it either, and one silent command alone would
// report an absent card for one sitting in the slot.
//
// Roughly 40ms, against the 80-150ms a successful `init` takes on the
// same board (it varies with how long the card spends answering
// `ACMD41`). Almost none of that 40ms is the wait for an answer, which
// times out in microseconds -- it is the controller's bring-up before
// any command is sent at all: the power domain, the clock, and their
// settling delays. So there is no faster check to reach for, which is
// the other thing worth seeing here.

use core::fmt::Write;

use rpi_hal::mailbox::Mailbox;
use rpi_hal::sd::{Block, Error, Sd};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::{halt, pac};

/// `INTERRUPT` error bits, most significant first, with the datasheet's
/// names. Printed for whichever are set: which error fired is the whole
/// question here, and `{e:?}` gives only the raw word.
const INTERRUPT_BITS: [(u32, &str); 9] = [
    (1 << 24, "ACMD_ERR"),
    (1 << 22, "DEND_ERR"),
    (1 << 21, "DCRC_ERR"),
    (1 << 20, "DTO_ERR"),
    (1 << 19, "CBAD_ERR"),
    (1 << 18, "CEND_ERR"),
    (1 << 17, "CCRC_ERR"),
    (1 << 16, "CTO_ERR"),
    (1 << 15, "ERR"),
];

/// `STATUS` bits worth naming — the inhibits and the physical line
/// levels, which together say whether the controller thinks something is
/// in flight and whether anything is holding the bus down.
const STATUS_BITS: [(u32, &str); 5] = [
    (1 << 0, "CMD_INHIBIT"),
    (1 << 1, "DAT_INHIBIT"),
    (1 << 2, "DAT_ACTIVE"),
    (1 << 24, "CMD_LEVEL"),
    (1 << 9, "READ_TRANSFER"),
];

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    let _ = writeln!(uart, "\r\nSD card presence probe");
    let _ = writeln!(
        uart,
        "Pull the card out or push it in, then press a key to probe."
    );
    let _ = writeln!(
        uart,
        "The card is only read when a probe succeeds, so removing it is safe."
    );

    let mut probe = 0u32;
    loop {
        let _ = writeln!(uart, "\r\n-- press a key --");
        let _ = uart.read_byte();
        probe += 1;

        // A fresh controller handle per attempt: `Sd::init` takes it by
        // value and a failed attempt doesn't hand it back. Sound here for
        // the reason `steal_emmc` documents -- nothing else in this
        // program touches EMMC, and any `Sd` from an earlier probe has
        // been dropped by now.
        let emmc = unsafe { Sd::steal_emmc() };

        let started = timer.now_micros();
        let result = Sd::init(&peripherals.GPIO, emmc, &mut mailbox, &timer);
        let elapsed = timer.now_micros() - started;

        match result {
            Ok(sd) => {
                let _ = writeln!(
                    uart,
                    "probe {probe}: card present, initialized in {elapsed}us ({}, {}-bit bus)",
                    if sd.high_capacity() {
                        "SDHC/SDXC"
                    } else {
                        "SDSC"
                    },
                    if sd.four_bit_bus() { 4 } else { 1 }
                );

                // Prove it is really usable, not merely identified.
                let mut block: Block = [0; 512];
                match sd.read_block(0, &mut block, &timer) {
                    Ok(()) => {
                        let _ = writeln!(
                            uart,
                            "  block 0 signature: {:02x} {:02x}{}",
                            block[510],
                            block[511],
                            if block[510..] == [0x55, 0xaa] {
                                " (0x55AA)"
                            } else {
                                " (unpartitioned card, or a blank region)"
                            }
                        );
                    }
                    Err(e) => {
                        let _ = writeln!(uart, "  block 0 read failed: {e:?}");
                    }
                }
            }
            Err(e) => {
                let _ = writeln!(uart, "probe {probe}: failed after {elapsed}us -- {e:?}");
                explain(&mut uart, e);
            }
        }
    }
}

/// Prints what an [`Error`] says about the controller, decoded.
///
/// The registers are already in the error for the two variants that
/// capture them; the rest have nothing to add beyond their own name, and
/// say so rather than being silently skipped.
fn explain(uart: &mut Uart, error: Error) {
    match error {
        Error::NoCard => {
            let _ = writeln!(
                uart,
                "  the slot is empty -- CMD8 went unanswered, and so did the CMD55 sent to check"
            );
        }
        Error::CardError { interrupt, command } => {
            let _ = writeln!(uart, "  command:   0x{command:08x} (CMD{})", command >> 24);
            print_bits(uart, "  INTERRUPT", interrupt, &INTERRUPT_BITS);
            if interrupt & (1 << 16) != 0 {
                let _ = writeln!(
                    uart,
                    "  CTO_ERR is the command timing out -- nothing on the bus answered."
                );
            }
        }
        Error::WaitTimeout {
            waiting_for,
            interrupt,
            status,
            command,
        } => {
            let _ = writeln!(
                uart,
                "  command:   0x{command:08x} (CMD{}), waiting for 0x{waiting_for:08x}",
                command >> 24
            );
            print_bits(uart, "  INTERRUPT", interrupt, &INTERRUPT_BITS);
            print_bits(uart, "  STATUS   ", status, &STATUS_BITS);
        }
        Error::LinesNotIdleHigh {
            cmd_line_high,
            data_lines_high,
        } => {
            let _ = writeln!(
                uart,
                "  CMD {}, DAT0-3 0b{data_lines_high:04b} -- a line is being held low",
                if cmd_line_high { "high" } else { "LOW" }
            );
        }
        Error::Timeout => {
            let _ = writeln!(
                uart,
                "  a bounded wait expired with no controller state captured"
            );
        }
        other => {
            let _ = writeln!(uart, "  no controller state carried by {other:?}");
        }
    }
}

/// Prints `value` in hex followed by the names of whichever `bits` are
/// set in it, or `(none set)`.
fn print_bits(uart: &mut Uart, label: &str, value: u32, bits: &[(u32, &str)]) {
    let _ = write!(uart, "{label}: 0x{value:08x}");
    let mut any = false;
    for (mask, name) in bits {
        if value & mask != 0 {
            let _ = write!(uart, "{}{name}", if any { " | " } else { "  " });
            any = true;
        }
    }
    if !any {
        let _ = write!(uart, "  (none of the named bits set)");
    }
    let _ = writeln!(uart);
}
