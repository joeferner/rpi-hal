//! The host side of the rpi-loader wire protocol.
//!
//! The loader stays resident on the device and services a command at a
//! time, so this is a connection to an already-running agent rather than
//! a one-shot uploader: every invocation re-handshakes, issues its
//! command, and leaves the device back in its command loop.
//!
//! Protocol (the device side lives in this project's `firmware` package):
//!
//! ```text
//! Handshake: host sends HELLO (b"RPIL"); device answers ACK (b"LIPR")
//! plus a version byte. The device answers a fresh HELLO from its command
//! loop too, which is how a new invocation reconnects mid-session. The
//! 4-byte magics (rather than a single byte) keep a boot-time electrical
//! transient from false-matching.
//!
//! Command byte, then per command:
//!   MEM_WRITE  [total,chunk,addr,crc u32 LE] -> OK/FAIL; then chunks
//!              [crc u32][bytes] each OK/FAIL (resend on FAIL); then a
//!              final OK/FAIL for the whole-image CRC.
//!   SET_BAUD   [baud u32 LE] -> device ACKs at the current baud then
//!              switches; host matches on OK.
//!   EXEC       [addr u32 LE] -> OK then the device jumps / FAIL.
//!   SD_LIST    [path] -> OK / FAIL+errcode; on OK a device->host stream.
//!   SD_READ    [path] -> OK / FAIL+errcode; on OK a device->host stream.
//!   SD_WRITE   [path][total,chunk u32 LE] -> OK / FAIL+errcode; on OK
//!              host->device chunks; then a final OK / FAIL+errcode.
//!   SD_DELETE  [path] -> OK / FAIL+errcode.
//!   SD_MKDIR   [path] -> OK / FAIL+errcode.
//!
//! A path is a u16 LE length followed by that many UTF-8 bytes. A
//! device->host stream is [total_len,chunk_size u32 LE] then chunks
//! [crc u32][bytes], each ACKed OK/FAIL by the host (device resends on
//! FAIL) -- the mirror of the host->device chunk flow.
//! ```

use std::io::{ErrorKind, Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
use serialport::{ClearBuffer, SerialPort};

/// Magic the host sends to open (or reopen) a session.
const HELLO: &[u8; 4] = b"RPIL";
/// Magic the device answers a [`HELLO`] with, followed by a version byte.
const ACK: &[u8; 4] = b"LIPR";
/// Protocol version this client speaks; a mismatch is a warning, not an
/// error, so a newer device can still be driven for the parts that match.
const PROTOCOL_VERSION: u8 = 1;
/// Status byte meaning the device accepted the last thing it was sent.
const OK: u8 = 1;
/// Status byte meaning it did not.
const FAIL: u8 = 0;
/// Payload bytes per chunk in both directions. The device rejects a
/// header asking for more than it can buffer, so this must stay at or
/// below the firmware's own stream chunk size.
const CHUNK_SIZE: usize = 4096;

/// Write a blob to a memory address (no jump).
const CMD_MEM_WRITE: u8 = 1;
/// Switch the link to a different baud.
const CMD_SET_BAUD: u8 = 2;
/// Jump to an address already in memory.
const CMD_EXEC: u8 = 3;
/// List a directory on the SD card's FAT boot partition.
const CMD_SD_LIST: u8 = 4;
/// Stream a file off the SD card.
const CMD_SD_READ: u8 = 5;
/// Stream a file onto the SD card.
const CMD_SD_WRITE: u8 = 6;
/// Delete a file from the SD card.
const CMD_SD_DELETE: u8 = 7;
/// Create a directory on the SD card.
const CMD_SD_MKDIR: u8 = 8;

/// Baud the handshake and terminal always run at (matches the device's
/// UART bring-up and a loaded kernel's default). The bulk transfers
/// negotiate up from here and always drop back before returning.
pub const BASE_BAUD: u32 = 115_200;
/// Default transfer baud. 1_500_000 is the sweet spot on this hardware:
/// it lands on an exact PL011 divisor (48MHz/(16*1.5M) = 2.0) and is ~13x
/// faster than the base rate. Lower it with `--baud` if a marginal cable
/// or long wiring corrupts chunks faster than the retry loop can recover.
pub const DEFAULT_BAUD: u32 = 1_500_000;
/// How long a single read waits before it counts as a timeout.
const RESPONSE_TIMEOUT: Duration = Duration::from_secs(2);
/// How many times a single chunk is resent before the transfer gives up.
const MAX_CHUNK_RETRIES: u32 = 5;
/// Bringing the SD card up can take up to ~1s (the card's ACMD41
/// power-up poll), so an `sd-*` command's first status byte gets several
/// timeout windows.
const SD_STATUS_ATTEMPTS: u32 = 3;
/// How long the terminal keeps printing what the device already sent
/// after the exit key, before giving up on a device that never pauses.
const DRAIN_LIMIT: Duration = Duration::from_millis(500);

/// Names the error code that follows a leading [`FAIL`] when a command
/// can't start (or, for `sd-write`, can't commit).
fn err_name(code: u8) -> String {
    match code {
        1 => "SD bring-up failed".into(),
        2 => "no such file or directory".into(),
        3 => "filesystem error".into(),
        4 => "directory listing too large".into(),
        5 => "bad path".into(),
        6 => "write failed".into(),
        other => format!("error code {other}"),
    }
}

/// One entry of an `sd-list` listing.
pub struct Entry {
    /// Whether the entry is a directory rather than a file.
    pub is_dir: bool,
    /// Size in bytes as the device reported it.
    pub size: u64,
    /// Entry name within the listed directory.
    pub name: String,
}

/// An open connection to the loader.
pub struct Link {
    /// The serial port itself.
    port: Box<dyn SerialPort>,
    /// Set by the Ctrl-C handler. Every loop that could otherwise block
    /// forever polls this: installing a handler replaces the default
    /// "SIGINT kills the process" behaviour, so without these checks a
    /// handshake against an unpowered board would be unquittable.
    interrupted: Arc<AtomicBool>,
}

impl Link {
    /// Opens `device` at [`BASE_BAUD`] without handshaking.
    pub fn open(device: &str, interrupted: Arc<AtomicBool>) -> Result<Self> {
        let port = serialport::new(device, BASE_BAUD)
            .timeout(RESPONSE_TIMEOUT)
            .open()
            .with_context(|| format!("opening {device}"))?;
        Ok(Self { port, interrupted })
    }

    /// Whether Ctrl-C has been pressed since the process started.
    pub fn interrupted(&self) -> bool {
        self.interrupted.load(Ordering::SeqCst)
    }

    /// Fails if Ctrl-C has been pressed, so a retry loop can unwind.
    fn check_interrupt(&self) -> Result<()> {
        if self.interrupted() {
            bail!("interrupted");
        }
        Ok(())
    }

    /// Reads exactly `n` bytes, or `None` if the port times out mid-read.
    ///
    /// Loops over the port so a large chunk that arrives in pieces still
    /// comes back whole; only a real stall (a full timeout with no
    /// further bytes) returns `None`.
    fn read_exact_or_timeout(&mut self, n: usize) -> Result<Option<Vec<u8>>> {
        let mut buf = vec![0u8; n];
        let mut filled = 0;
        while filled < n {
            match self.port.read(&mut buf[filled..]) {
                Ok(0) => return Ok(None),
                Ok(got) => filled += got,
                Err(e) if e.kind() == ErrorKind::TimedOut => return Ok(None),
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(e) => return Err(e).context("reading from the serial port"),
            }
        }
        Ok(Some(buf))
    }

    /// Reads one raw protocol status byte, or `None` after `attempts`
    /// timeout windows.
    ///
    /// Unlike the handshake, the device stays silent (no log text) around
    /// status bytes, so the next byte is the status — no
    /// printable-filtering needed here.
    fn read_status(&mut self, attempts: u32) -> Result<Option<u8>> {
        for _ in 0..attempts {
            if let Some(b) = self.read_exact_or_timeout(1)? {
                return Ok(Some(b[0]));
            }
        }
        Ok(None)
    }

    /// Reads the error code that follows a leading [`FAIL`] and names it.
    fn fail_reason(&mut self) -> Result<String> {
        Ok(match self.read_status(1)? {
            Some(code) => err_name(code),
            None => "timed out reading error code".into(),
        })
    }

    /// Scans for the 4-byte [`ACK`] magic, then reads the version byte
    /// after it. `None` on timeout.
    ///
    /// Printable bytes seen before the match are echoed to stdout —
    /// that's the device's boot banner. A boot-time electrical transient
    /// can produce a stray non-printable byte that isn't a real response,
    /// so this requires the full ACK match rather than trusting one byte.
    fn wait_for_ack_and_version(&mut self) -> Result<Option<u8>> {
        let mut matched = 0;
        while matched < ACK.len() {
            let Some(b) = self.read_exact_or_timeout(1)? else {
                return Ok(None);
            };
            let value = b[0];
            if value == ACK[matched] {
                matched += 1;
            } else if value == ACK[0] {
                matched = 1;
            } else {
                matched = 0;
                if (0x20..=0x7E).contains(&value) || value == b'\n' || value == b'\r' {
                    let mut out = std::io::stdout();
                    out.write_all(&b)?;
                    out.flush()?;
                }
            }
        }
        Ok(self.read_exact_or_timeout(1)?.map(|b| b[0]))
    }

    /// Establishes (or re-establishes) contact with the loader.
    ///
    /// Sends HELLO until the device answers ACK + version — this works
    /// whether the Pi just booted (blocking on its power-on handshake) or
    /// is already sitting in its command loop (which re-greets on HELLO).
    /// Drops any extra greeting so a later status read can't misparse it.
    pub fn handshake(&mut self) -> Result<u8> {
        self.port.clear(ClearBuffer::All).ok();

        eprintln!("Connecting to rpi-loader (sending HELLO)...");
        let version = loop {
            self.check_interrupt()?;
            self.write_all(HELLO)?;
            if let Some(version) = self.wait_for_ack_and_version()? {
                break version;
            }
        };
        if version != PROTOCOL_VERSION {
            eprintln!("Warning: device protocol version {version}, expected {PROTOCOL_VERSION}");
        }
        // A reconnect can race an extra HELLO into a second greeting;
        // clear it so it isn't mistaken for a command's status byte.
        sleep(Duration::from_millis(20));
        self.port.clear(ClearBuffer::Input).ok();
        Ok(version)
    }

    /// Switches the link to `baud` (up for a bulk transfer, or back
    /// down).
    ///
    /// Sends `CMD_SET_BAUD` + the target (u32 LE). The device ACKs at the
    /// *current* baud then switches; on OK the host switches to match. On
    /// FAIL (the device can't form that divisor) or timeout, both ends
    /// stay put. Returns the baud actually in effect afterward.
    pub fn negotiate_baud(&mut self, baud: u32) -> Result<u32> {
        let current = self.port.baud_rate().context("reading the current baud")?;
        if current == baud {
            return Ok(baud);
        }
        let mut packet = vec![CMD_SET_BAUD];
        packet.extend_from_slice(&baud.to_le_bytes());
        self.write_all(&packet)?;
        if self.read_status(1)? != Some(OK) {
            eprintln!("Device declined {baud} baud; staying at {current}");
            return Ok(current);
        }

        // The device has ACKed at the old baud and is now switching. Match
        // it, then drop any glitch the divisor change latched and let the
        // new rate settle before the first byte sent at it.
        self.port
            .set_baud_rate(baud)
            .with_context(|| format!("switching the host to {baud} baud"))?;
        sleep(Duration::from_millis(50));
        self.port.clear(ClearBuffer::Input).ok();
        Ok(baud)
    }

    /// Streams `data` to the device as CRC-checked chunks
    /// (host→device).
    ///
    /// Each chunk is `[crc u32 LE][bytes]`; the device ACKs OK/FAIL and
    /// the host resends the same chunk on FAIL, up to
    /// [`MAX_CHUNK_RETRIES`] — the self-healing transfer the loader's
    /// protocol exists to provide.
    fn send_chunked(&mut self, data: &[u8]) -> Result<()> {
        let total = data.len();
        for (offset, chunk) in (0..).step_by(CHUNK_SIZE).zip(data.chunks(CHUNK_SIZE)) {
            let mut packet = crc32(chunk).to_le_bytes().to_vec();
            packet.extend_from_slice(chunk);

            let mut sent = false;
            for attempt in 1..=MAX_CHUNK_RETRIES {
                self.check_interrupt()?;
                self.write_all(&packet)?;
                if self.read_status(1)? == Some(OK) {
                    sent = true;
                    break;
                }
                eprintln!(
                    "  chunk at {offset} failed/timed out, retry {attempt}/{MAX_CHUNK_RETRIES}"
                );
            }
            if !sent {
                bail!("giving up on chunk at {offset} after {MAX_CHUNK_RETRIES} retries");
            }
            eprintln!("  {}/{total}", (offset + CHUNK_SIZE).min(total));
        }
        Ok(())
    }

    /// Receives a CRC-checked stream from the device (device→host).
    ///
    /// Reads the `[total_len, chunk_size]` header, then each chunk
    /// `[crc u32 LE][bytes]`; the host verifies the CRC and ACKs OK/FAIL,
    /// prompting the device to resend on FAIL. The mirror of
    /// [`Link::send_chunked`].
    fn recv_chunked(&mut self) -> Result<Vec<u8>> {
        let header = self
            .read_exact_or_timeout(8)?
            .ok_or_else(|| anyhow!("timed out waiting for stream header"))?;
        let total_len = u32::from_le_bytes(header[0..4].try_into().unwrap()) as usize;
        let chunk_size = u32::from_le_bytes(header[4..8].try_into().unwrap()) as usize;

        let mut out = Vec::with_capacity(total_len);
        while out.len() < total_len {
            let this = chunk_size.min(total_len - out.len());
            let mut received = false;
            for attempt in 1..=MAX_CHUNK_RETRIES {
                self.check_interrupt()?;
                let crc_bytes = self.read_exact_or_timeout(4)?;
                let data = self.read_exact_or_timeout(this)?;
                let (Some(crc_bytes), Some(data)) = (crc_bytes, data) else {
                    // A mid-chunk timeout means the framing is lost —
                    // there's no safe point to resync from, so give up on
                    // the stream.
                    bail!("timed out mid-stream");
                };
                if crc32(&data) == u32::from_le_bytes(crc_bytes.try_into().unwrap()) {
                    self.write_all(&[OK])?;
                    out.extend_from_slice(&data);
                    received = true;
                    break;
                }
                self.write_all(&[FAIL])?;
                eprintln!(
                    "  chunk at {} bad CRC, retry {attempt}/{MAX_CHUNK_RETRIES}",
                    out.len()
                );
            }
            if !received {
                bail!(
                    "giving up on chunk at {} after {MAX_CHUNK_RETRIES} retries",
                    out.len()
                );
            }
        }
        Ok(out)
    }

    /// Sends a path argument: u16 LE length then UTF-8 bytes.
    fn send_path(&mut self, path: &str) -> Result<()> {
        let encoded = path.as_bytes();
        let len = u16::try_from(encoded.len()).context("path is too long for the protocol")?;
        let mut packet = len.to_le_bytes().to_vec();
        packet.extend_from_slice(encoded);
        self.write_all(&packet)
    }

    /// Writes every byte of `data` to the port.
    fn write_all(&mut self, data: &[u8]) -> Result<()> {
        self.port
            .write_all(data)
            .context("writing to the serial port")
    }

    /// Writes `data` to memory at `addr` (no jump).
    pub fn mem_write(&mut self, addr: u32, data: &[u8]) -> Result<()> {
        let mut header = vec![CMD_MEM_WRITE];
        header.extend_from_slice(&(data.len() as u32).to_le_bytes());
        header.extend_from_slice(&(CHUNK_SIZE as u32).to_le_bytes());
        header.extend_from_slice(&addr.to_le_bytes());
        header.extend_from_slice(&crc32(data).to_le_bytes());
        self.write_all(&header)?;
        if self.read_status(1)? != Some(OK) {
            bail!("device rejected header (bad size/address?)");
        }
        eprintln!("Sending {} bytes to {addr:#x}...", data.len());
        self.send_chunked(data)?;
        if self.read_status(1)? != Some(OK) {
            bail!("device reported overall checksum mismatch");
        }
        Ok(())
    }

    /// Jumps to `addr`. The device does not reply again after this.
    pub fn exec(&mut self, addr: u32) -> Result<()> {
        let mut packet = vec![CMD_EXEC];
        packet.extend_from_slice(&addr.to_le_bytes());
        self.write_all(&packet)?;
        if self.read_status(1)? != Some(OK) {
            bail!("device refused to exec {addr:#x} (bad address?)");
        }
        Ok(())
    }

    /// Starts an `sd-*` command that takes a single path, and waits for
    /// its leading status byte.
    fn start_sd_command(&mut self, command: u8, path: &str, what: &str) -> Result<()> {
        self.write_all(&[command])?;
        self.send_path(path)?;
        if self.read_status(SD_STATUS_ATTEMPTS)? != Some(OK) {
            let reason = self.fail_reason()?;
            bail!("{what} failed: {reason}");
        }
        Ok(())
    }

    /// Lists a directory on the SD card's FAT boot partition.
    pub fn sd_list(&mut self, path: &str) -> Result<Vec<Entry>> {
        self.start_sd_command(CMD_SD_LIST, path, "sd-list")?;
        let listing = self.recv_chunked()?;
        let listing = String::from_utf8_lossy(&listing);
        listing
            .lines()
            .map(|line| {
                let mut fields = line.splitn(3, '\t');
                let (Some(kind), Some(size), Some(name)) =
                    (fields.next(), fields.next(), fields.next())
                else {
                    bail!("malformed listing line from the device: {line:?}");
                };
                Ok(Entry {
                    is_dir: kind == "D",
                    size: size
                        .parse()
                        .context("listing entry has a non-numeric size")?,
                    name: name.to_string(),
                })
            })
            .collect()
    }

    /// Copies `remote` off the SD card and returns its contents.
    pub fn sd_read(&mut self, remote: &str) -> Result<Vec<u8>> {
        self.start_sd_command(CMD_SD_READ, remote, "sd-read")?;
        self.recv_chunked()
    }

    /// Writes `data` onto the SD card as `remote`, creating or
    /// truncating it.
    pub fn sd_write(&mut self, remote: &str, data: &[u8]) -> Result<()> {
        self.write_all(&[CMD_SD_WRITE])?;
        self.send_path(remote)?;
        let mut header = (data.len() as u32).to_le_bytes().to_vec();
        header.extend_from_slice(&(CHUNK_SIZE as u32).to_le_bytes());
        self.write_all(&header)?;
        if self.read_status(SD_STATUS_ATTEMPTS)? != Some(OK) {
            let reason = self.fail_reason()?;
            bail!("sd-write failed: {reason}");
        }
        eprintln!("Sending {} bytes -> {remote}...", data.len());
        self.send_chunked(data)?;
        if self.read_status(SD_STATUS_ATTEMPTS)? != Some(OK) {
            let reason = self.fail_reason()?;
            bail!("sd-write did not commit: {reason}");
        }
        Ok(())
    }

    /// Deletes `remote` from the SD card.
    pub fn sd_delete(&mut self, remote: &str) -> Result<()> {
        self.start_sd_command(CMD_SD_DELETE, remote, "sd-delete")
    }

    /// Creates the directory `remote` on the SD card. A single level —
    /// the parent directories must already exist.
    pub fn sd_mkdir(&mut self, remote: &str) -> Result<()> {
        self.start_sd_command(CMD_SD_MKDIR, remote, "sd-mkdir")
    }

    /// Acts as a bidirectional passthrough terminal: what the device
    /// sends goes to stdout, and what is typed goes to the device.
    /// Returns when [`ESCAPE`] is pressed.
    ///
    /// The two directions block independently — a keystroke can arrive
    /// while nothing is coming back, and vice versa — so the device→stdout
    /// half runs on its own thread over a cloned port handle, and the
    /// keyboard is read here.
    pub fn terminal(&mut self) -> Result<()> {
        let mut from_device = self
            .port
            .try_clone()
            .context("cloning the serial port for the terminal")?;
        // A shorter timeout than the protocol's: nothing here waits on a
        // reply, and this is how quickly the reader notices it should
        // stop once the terminal is being left.
        from_device
            .set_timeout(Duration::from_millis(100))
            .context("setting the terminal read timeout")?;

        let raw = RawMode::enable();
        if raw.active {
            eprintln!(
                "Entering terminal mode. Ctrl-] exits; everything else, \
                 Ctrl-C included, goes to the device.\r"
            );
        } else {
            // Without a terminal on stdin there is no raw mode and no
            // signal handling to bypass, so Ctrl-C keeps its usual job.
            eprintln!("Entering terminal mode (stdin is not a terminal; Ctrl-C exits).");
        }

        let finished = Arc::new(AtomicBool::new(false));
        let reader_finished = Arc::clone(&finished);
        let reader_interrupted = Arc::clone(&self.interrupted);
        let reader = std::thread::spawn(move || {
            let mut buf = [0u8; 256];
            let mut out = std::io::stdout();
            while !reader_finished.load(Ordering::SeqCst)
                && !reader_interrupted.load(Ordering::SeqCst)
            {
                match from_device.read(&mut buf) {
                    Ok(0) => {}
                    Ok(n) => {
                        if out.write_all(&buf[..n]).is_err() || out.flush().is_err() {
                            break;
                        }
                    }
                    Err(e) if e.kind() == ErrorKind::TimedOut => {}
                    Err(e) if e.kind() == ErrorKind::Interrupted => {}
                    // The port went away (cable unplugged, adapter reset).
                    // Nothing to report from a thread; the keyboard half
                    // sees `finished` and stops waiting.
                    Err(_) => break,
                }
            }

            // Whatever the device already sent is worth showing even
            // though the terminal is being left: the exit is a keystroke
            // on this end, and the device knew nothing about it when it
            // sent those bytes. Draining until it goes quiet for one
            // timeout window bounds this — a device talking continuously
            // stops it after DRAIN_LIMIT rather than holding the exit
            // open indefinitely.
            let deadline = Instant::now() + DRAIN_LIMIT;
            while Instant::now() < deadline {
                match from_device.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => {
                        if out.write_all(&buf[..n]).is_err() || out.flush().is_err() {
                            break;
                        }
                    }
                    _ => break,
                }
            }
            reader_finished.store(true, Ordering::SeqCst);
        });

        let result = self.forward_keyboard(&finished);

        finished.store(true, Ordering::SeqCst);
        let _ = reader.join();
        // Restore the terminal before printing, or the newline lands
        // without a carriage return and the message starts mid-line.
        drop(raw);
        eprintln!();
        eprintln!("Exiting.");
        result
    }

    /// Reads the keyboard and writes it to the device until [`ESCAPE`],
    /// end of input, or Ctrl-C.
    fn forward_keyboard(&mut self, finished: &AtomicBool) -> Result<()> {
        let mut stdin = std::io::stdin().lock();
        let mut buf = [0u8; 256];
        loop {
            if self.interrupted() || finished.load(Ordering::SeqCst) {
                return Ok(());
            }
            let n = match stdin.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => n,
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(e) => return Err(e).context("reading the keyboard"),
            };
            if let Some(at) = buf[..n].iter().position(|&b| b == ESCAPE) {
                // Whatever was typed ahead of the escape was meant for the
                // device; only the escape itself is swallowed.
                self.write_all(&buf[..at])?;
                return Ok(());
            }
            self.write_all(&buf[..n])?;
        }

        // Input ended without an escape — stdin was a pipe or a file
        // rather than a keyboard. The device may still have plenty to
        // say, so keep displaying it until Ctrl-C or the port dies.
        while !self.interrupted() && !finished.load(Ordering::SeqCst) {
            sleep(Duration::from_millis(100));
        }
        Ok(())
    }
}

/// Key that leaves terminal mode: Ctrl-] (0x1D), the telnet convention.
///
/// Ctrl-C cannot be the way out once the terminal is bidirectional — the
/// point of raw mode is that it reaches the device, which is how you
/// interrupt something running *there*. Ctrl-] is a single keystroke
/// needing no state machine, unlike the `~.` and `Ctrl-A X` sequences,
/// and nothing on the device side is likely to want it.
const ESCAPE: u8 = 0x1D;

/// Puts the host terminal into raw mode for as long as it is alive, so
/// keystrokes reach the device one at a time, without local echo, and
/// without the line discipline turning Ctrl-C into a signal.
///
/// A guard rather than a pair of calls because every exit path has to
/// restore the terminal — an early return, a `?`, or a panic unwinding
/// past it. A shell left in raw mode has no echo and no working Enter
/// key, which is a miserable thing to hand back to someone.
struct RawMode {
    /// Whether raw mode was actually entered. It is not when stdin is a
    /// pipe or a file, and there is then nothing to restore.
    active: bool,
}

impl RawMode {
    /// Enters raw mode if stdin is a terminal.
    fn enable() -> Self {
        Self {
            active: enable_raw_mode().is_ok(),
        }
    }
}

impl Drop for RawMode {
    fn drop(&mut self) {
        if self.active {
            let _ = disable_raw_mode();
        }
    }
}

/// CRC-32/ISO-HDLC over `data` — the same algorithm the device computes,
/// so the two agree by construction.
fn crc32(data: &[u8]) -> u32 {
    let mut hasher = crc32fast::Hasher::new();
    hasher.update(data);
    hasher.finalize()
}
