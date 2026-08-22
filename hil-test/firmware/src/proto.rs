//! The control-interface wire format.
//!
//! Binary, request/response, one exchange per USB bulk packet. Deliberately
//! not a text protocol: this channel also has to carry capture buffers, and
//! hex-encoding those doubles their size for no benefit when the peer is
//! Python speaking libusb rather than a human at a terminal.
//!
//! `PROTOCOL.md` alongside this crate is the authoritative description; keep
//! the two in step.

/// Version of this wire format, returned by [`Cmd::Hello`]. The runner
/// refuses to drive a fixture whose version it does not know, rather than
/// guessing at a mismatched layout.
pub const PROTOCOL_VERSION: u8 = 1;

/// Largest request or response body. One full-speed bulk packet minus the
/// two header bytes, so every exchange is a single transfer.
pub const MAX_BODY: usize = 62;

/// Request opcodes, the first byte of every request packet.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Cmd {
    /// Liveness check. No body, empty response body.
    Ping = 0x01,
    /// Identify the fixture: protocol version, board, capabilities, firmware
    /// version. The runner's first call, and what every skip decision keys
    /// off.
    Hello = 0x02,
    /// Release the console pins so a case can drive them, buffering nothing.
    ConsoleDetach = 0x10,
    /// Resume bridging the console pins.
    ConsoleAttach = 0x11,
    /// Report whether the bridge is attached, and at what baud.
    ConsoleStatus = 0x12,
    /// Sample the two console pins, so the host can witness what the board
    /// drove on them while the bridge was released.
    ConsolePins = 0x13,
    /// Drive the two console pins, so the board can read what the fixture put
    /// on them. Only valid while the console is detached.
    ConsoleDrive = 0x14,
    /// Start a marker-pin capture, discarding whatever the last one held.
    MarkerArm = 0x20,
    /// Report how much of the current capture is filled, and the timebase it
    /// is measured in.
    MarkerStatus = 0x21,
    /// Read a run of timestamps out of the capture buffer.
    MarkerRead = 0x22,
    /// Drive a pulse train on the marker pin, so the capture path can be
    /// tested without a board.
    MarkerPulse = 0x23,
}

impl Cmd {
    /// Decodes an opcode byte, returning `None` for anything unrecognised so
    /// the caller can answer [`Status::BadCommand`] rather than misparse.
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0x01 => Cmd::Ping,
            0x02 => Cmd::Hello,
            0x10 => Cmd::ConsoleDetach,
            0x11 => Cmd::ConsoleAttach,
            0x12 => Cmd::ConsoleStatus,
            0x13 => Cmd::ConsolePins,
            0x14 => Cmd::ConsoleDrive,
            0x20 => Cmd::MarkerArm,
            0x21 => Cmd::MarkerStatus,
            0x22 => Cmd::MarkerRead,
            0x23 => Cmd::MarkerPulse,
            _ => return None,
        })
    }
}

/// Response status, the first byte of every response packet.
///
/// The whole vocabulary is defined even though this build no longer returns
/// all of it: the values are wire format shared with the host, and a status
/// that vanishes from the enum when the last command using it starts working
/// is a status the next command has to reinvent — with a different number.
#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Status {
    /// Succeeded; body holds whatever the command returns.
    Ok = 0x00,
    /// Opcode not recognised by this firmware.
    BadCommand = 0x01,
    /// Opcode understood but the body was malformed or the wrong length.
    BadArgs = 0x02,
    /// Understood, but this fixture lacks the capability. Distinct from
    /// [`Status::BadCommand`] so the runner can skip rather than error.
    Unsupported = 0x03,
    /// Understood and well-formed, but not valid in the fixture's current
    /// state — driving the console pins while the bridge still owns them,
    /// for instance.
    ///
    /// Distinct from [`Status::BadArgs`] because the fix is different and so
    /// is the blame: bad arguments mean the caller built the request wrong
    /// and no amount of retrying helps, while this one means the same request
    /// is correct a moment later. Collapsing them would have a sequencing bug
    /// present as a malformed packet, sending whoever debugs it to read the
    /// codec.
    BadState = 0x04,
}

/// Firmware version reported by [`Cmd::Hello`], as major/minor/patch.
///
/// Bumped when behaviour the host can observe changes without the wire
/// layout changing — the console handoff going from `UNSUPPORTED` to working
/// is exactly that, and the version is the only way a runner can tell a
/// fixture that predates it from one that has it.
pub const FIRMWARE_VERSION: [u8; 3] = [0, 4, 0];
