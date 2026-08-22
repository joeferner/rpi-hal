//! Per-board identity, capabilities and pin assignment.
//!
//! The fixture never assumes what it can do — it reports it, and the runner
//! skips cases whose needs are unmet. So a reduced board like the Pico is a
//! smaller capability set, not a broken rig.

/// Identifies which fixture board this firmware is running on. Reported by
/// `HELLO` so the runner can cross-check `bench.toml`.
///
/// Every board is listed regardless of which one this build targets, for
/// the same reason as the capability bits: the values go on the wire, so
/// they are allocated once rather than per build.
#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BoardId {
    /// RP2040 on a Raspberry Pi Pico. 26 GPIO, so no full-header shadow.
    Rp2040Pico = 1,
    /// RP2350B on an Olimex PICO2-XL. All 48 GPIO.
    Rp2350Pico2Xl = 2,
}

/// One bit per thing the fixture can do, reported by `HELLO`.
///
/// The runner treats an absent bit as "skip with a reason", never as a
/// failure, so adding hardware to the bench widens coverage without
/// touching any case.
///
/// The whole vocabulary is defined here even though this build only claims
/// one of them, because the bit positions are wire format shared with the
/// host — they have to be assigned once and never renumbered, not invented
/// as each capability lands.
#[allow(dead_code)]
pub mod caps {
    /// Bridges the board's console UART to the host as a serial port.
    pub const CONSOLE_BRIDGE: u32 = 1 << 0;
    /// Drives and reads the board's GPIO header 1:1.
    pub const GPIO_SHADOW: u32 = 1 << 1;
    /// Timestamps marker-pin edges against the fixture's clock.
    pub const MARKER_TIMESTAMP: u32 = 1 << 2;
    /// Plays the SPI slave role in any of the four modes.
    pub const SPI_SLAVE: u32 = 1 << 3;
    /// Plays a programmable I2C slave, including NAK and clock stretch.
    pub const I2C_SLAVE: u32 = 1 << 4;
    /// Samples several lines at once into a capture buffer.
    pub const LOGIC_CAPTURE: u32 = 1 << 5;
    /// Receives I2S and returns the frames bit-exactly.
    pub const I2S_CAPTURE: u32 = 1 << 6;
    /// Samples the analog audio output.
    pub const AUDIO_ADC: u32 = 1 << 7;
    /// Switches the board's 5V supply, i.e. can power-cycle it.
    pub const POWER_SWITCH: u32 = 1 << 8;
    /// Reads the board's 3V3 rail, so a cold boot can be confirmed.
    pub const RAIL_SENSE: u32 = 1 << 9;
    /// Measures current into the board's rail.
    pub const CURRENT_SENSE: u32 = 1 << 10;
    /// Switches VBUS to individual USB devices.
    pub const USB_VBUS_SWITCH: u32 = 1 << 11;
    /// Pulls the board's `RUN` pad low for a warm reset.
    pub const RUN_RESET: u32 = 1 << 12;
}

/// Which board this build targets.
#[cfg(feature = "rp2040")]
pub const BOARD: BoardId = BoardId::Rp2040Pico;
/// Which board this build targets.
#[cfg(feature = "rp235x")]
pub const BOARD: BoardId = BoardId::Rp2350Pico2Xl;

/// What this build can actually do.
///
/// Everything else is deliberately absent rather than optimistically claimed —
/// a capability bit that lies is worse than one that is missing, because the
/// runner stops skipping and starts reporting false failures.
///
/// `MARKER_TIMESTAMP` is claimed and `GPIO_SHADOW` is not, which is worth
/// contrasting because both are implemented on the same two-wire breadboard.
/// The marker bit describes exactly what this fixture does: one designated pin
/// whose edges are timestamped, which is all the convention ever asks for.
/// `GPIO_SHADOW` means the whole 40-pin header 1:1, and a fixture that drives
/// two of those pins claiming it would have the runner stop skipping the cases
/// that need the other twenty-six.
pub const CAPABILITIES: u32 = caps::CONSOLE_BRIDGE | caps::MARKER_TIMESTAMP;

/// Baud the console bridge starts at, matching the loader's idle rate. The
/// host overrides it through the CDC interface's line coding, so this only
/// governs the window before the host first opens the port.
pub const CONSOLE_DEFAULT_BAUD: u32 = 115_200;
