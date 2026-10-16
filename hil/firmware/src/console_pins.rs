//! Time-sharing the console pins between the bridge and a case under test.
//!
//! GPIO14/15 are the only header pins on a BCM283x with UART alt functions,
//! so a case that wants to drive them has to borrow them from its own
//! console. This module is the fixture's half of that handoff: it moves
//! GP0/GP1 between the UART and SIO, and reads their levels while they are
//! released.
//!
//! The mux is moved rather than the `BufferedUart` being dropped and rebuilt.
//! Two reasons, and the second is the one that would bite:
//!
//! - The peripheral keeps its configuration, so reattaching restores the
//!   *exact* link that was torn down — including the baud the host set
//!   through CDC line coding, which a rebuild would reset to the default and
//!   silently garble a loader session negotiated up to 1.5 Mbaud.
//! - Rebuilding needs the pin and UART singletons back, which the bridge
//!   future owns for its whole lifetime. Stealing them to work around that
//!   would leave two owners of one peripheral for the sake of a register
//!   write either of them could have done.

use embassy_rp::pac;

/// Fixture pin carrying the board's RXD0, i.e. board GPIO15. Our transmit.
const TX_PIN: usize = 0;
/// Fixture pin carrying the board's TXD0, i.e. board GPIO14. Our receive.
const RX_PIN: usize = 1;

/// FUNCSEL that connects GP0/GP1 to UART0.
///
/// The RP2040 and RP2350 both put UART0 on function 2 for these pads, which
/// is the same number `embassy-rp` writes when it brings the UART up — so
/// reattaching restores byte-for-byte what its own init left there, rather
/// than an approximation of it.
const FUNCSEL_UART: u8 = 2;

/// FUNCSEL that hands a pad to SIO, so the fixture can read or drive it
/// directly. Named differently per chip in the PAC (`SIO_0` against
/// `SIOB_PROC_0`) but the same value on the wire.
#[cfg(feature = "rp2040")]
const FUNCSEL_SIO: u8 = pac::io::vals::Gpio0ctrlFuncsel::SIO_0 as u8;
/// FUNCSEL that hands a pad to SIO. See the `rp2040` arm above.
#[cfg(feature = "rp235x")]
const FUNCSEL_SIO: u8 = pac::io::vals::Gpio0ctrlFuncsel::SIOB_PROC_0 as u8;

/// Bank-0 SIO register index for GP0/GP1. Both chips address the low 32
/// pins as bank/group 0, so one constant covers them.
const SIO_BANK: usize = 0;

/// Releases GP0/GP1 from the UART, leaving them as high-impedance inputs.
///
/// The output enable is cleared *before* the mux moves, not after. Between
/// those two writes the pad follows whichever driver the funcsel names, so
/// doing it the other way round would briefly drive the board's console pins
/// from SIO's output register — whose contents nothing here has ever set.
pub fn release() {
    for pin in [TX_PIN, RX_PIN] {
        pac::SIO.gpio_oe(SIO_BANK).value_clr().write_value(1 << pin);
        pac::IO_BANK0
            .gpio(pin)
            .ctrl()
            .write(|w| w.set_funcsel(FUNCSEL_SIO));
        // The input buffer is already enabled — the UART's own init needs it
        // for RX — but this is what makes reading the pads a property of this
        // function rather than an inherited side effect of the bridge's
        // setup, which is where it would break if the bridge ever changed.
        pac::PADS_BANK0.gpio(pin).modify(|w| w.set_ie(true));
    }
}

/// Reconnects GP0/GP1 to the UART, resuming the bridge.
///
/// The UART was never reconfigured, so this restores the link at whatever
/// baud the host last asked for rather than at the default.
///
/// Drops any drive first. Once the funcsel names the UART, SIO's output
/// enable no longer reaches the pad and clearing it is redundant — but a
/// fixture that came back from a window still holding a stale output enable
/// would leave the next `release` briefly driving before anyone asked it to,
/// which is the failure this whole module is arranged to avoid.
pub fn reconnect() {
    for pin in [TX_PIN, RX_PIN] {
        pac::SIO.gpio_oe(SIO_BANK).value_clr().write_value(1 << pin);
        pac::IO_BANK0
            .gpio(pin)
            .ctrl()
            .write(|w| w.set_funcsel(FUNCSEL_UART));
    }
}

/// Drives, or stops driving, the two console pins.
///
/// `oe` selects which pins the fixture drives and `levels` what it drives
/// them to, both using the same bit assignment as [`levels`]: bit 0 the
/// board's GPIO14, bit 1 its GPIO15. A pin whose `oe` bit is clear goes
/// high-impedance, which is the resting state and how a caller hands the wire
/// back to the board.
///
/// Only meaningful once [`release`] has moved the pads to SIO; the caller is
/// responsible for that ordering, because the fixture cannot tell whether the
/// *board* has let go of its end and answering "yes, driving" into a pin the
/// board still owns would create exactly the contention the handshake exists
/// to prevent.
///
/// The level is written before the output enable, for the same reason
/// [`release`] clears the enable before moving the mux: enabling first would
/// drive whatever the output register happened to be holding, for as long as
/// it takes to reach the next instruction.
pub fn drive(oe: u8, levels: u8) {
    for (bit, pin) in [(0, RX_PIN), (1, TX_PIN)] {
        let mask = 1 << pin;
        if oe & (1 << bit) == 0 {
            pac::SIO.gpio_oe(SIO_BANK).value_clr().write_value(mask);
            continue;
        }
        if levels & (1 << bit) == 0 {
            pac::SIO.gpio_out(SIO_BANK).value_clr().write_value(mask);
        } else {
            pac::SIO.gpio_out(SIO_BANK).value_set().write_value(mask);
        }
        pac::SIO.gpio_oe(SIO_BANK).value_set().write_value(mask);
    }
}

/// Reads the two console pins as they are right now.
///
/// Bit 0 is the board's GPIO14 (its TXD0, our receive) and bit 1 its GPIO15
/// (its RXD0, our transmit). Named for the *board's* pins because that is
/// what a case asserts about; the fixture's own numbering is an
/// implementation detail of this wiring.
///
/// Valid whether or not the pins are released — `GPIO_IN` always reflects the
/// pad, whichever peripheral is muxed onto it. That is deliberate: a level
/// read that silently returned nothing while attached would make a
/// misordered handoff look like a dead pin rather than a sequencing bug.
pub fn levels() -> u8 {
    let raw = pac::SIO.gpio_in(SIO_BANK).read();
    let gpio14 = (raw >> RX_PIN) & 1;
    let gpio15 = (raw >> TX_PIN) & 1;
    (gpio14 | (gpio15 << 1)) as u8
}
