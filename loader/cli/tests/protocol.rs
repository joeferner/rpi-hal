//! Wire-protocol tests against a fake device on a pty.
//!
//! These run the real binary against a scripted device that speaks the
//! other half of the protocol, so they cover the parts a unit test cannot
//! reach: the byte-for-byte framing, the CRC retries, the order commands
//! are issued in, and the exit status. What they deliberately do not
//! cover is timing — a pty delivers bytes instantly and has no baud rate,
//! so the pacing that keeps a real PL011's RX FIFO from overflowing is
//! only ever proven on hardware.
//!
//! Each test scripts the exact exchange it expects rather than dispatching
//! on whatever arrives. That makes the command *order* part of the
//! assertion: `mem-write` negotiating up, transferring, and dropping back
//! to the base baud is the behaviour under test, not an incidental detail.

use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::OwnedFd;
use std::path::PathBuf;
use std::process::{Child, ChildStdin, Command, Output, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use nix::pty::openpty;
use nix::sys::signal::{kill, Signal};
use nix::sys::termios::{cfmakeraw, tcgetattr, tcsetattr, SetArg, SpecialCharacterIndices};
use nix::unistd::{ttyname, Pid};

/// Handshake magic the host sends.
const HELLO: &[u8; 4] = b"RPIL";
/// Magic the device answers with, followed by a version byte.
const ACK: &[u8; 4] = b"LIPR";
/// Protocol version the device claims.
const VERSION: u8 = 1;
/// Status byte for success.
const OK: u8 = 1;
/// Status byte for failure.
const FAIL: u8 = 0;
/// Payload bytes per chunk, matching the CLI and the firmware.
const CHUNK: usize = 4096;
/// Baud the link idles at.
const BASE_BAUD: u32 = 115_200;
/// Baud the bulk commands negotiate up to by default.
const FAST_BAUD: u32 = 1_500_000;
/// Key that leaves terminal mode: Ctrl-].
const ESCAPE: u8 = 0x1D;

const CMD_MEM_WRITE: u8 = 1;
const CMD_SET_BAUD: u8 = 2;
const CMD_EXEC: u8 = 3;
const CMD_SD_LIST: u8 = 4;
const CMD_SD_READ: u8 = 5;
const CMD_SD_WRITE: u8 = 6;
const CMD_SD_DELETE: u8 = 7;
const CMD_SD_MKDIR: u8 = 8;

/// Error code the device sends for a missing file.
const ERR_NOT_FOUND: u8 = 2;
/// Error code the device sends when a write cannot be committed.
const ERR_WRITE: u8 = 6;

/// How long the child gets before the watchdog kills it. Only ever
/// reached when something has already gone wrong; a passing test finishes
/// in well under a second.
const CHILD_TIMEOUT: Duration = Duration::from_secs(20);

/// CRC-32/ISO-HDLC, the same one both halves of the protocol use.
fn crc32(data: &[u8]) -> u32 {
    let mut hasher = crc32fast::Hasher::new();
    hasher.update(data);
    hasher.finalize()
}

/// A payload spanning three chunks, so the chunking itself is exercised
/// rather than a single-chunk special case.
fn payload() -> Vec<u8> {
    (0..10_000).map(|i| ((i * 7 + 3) % 256) as u8).collect()
}

/// Writes `data` to a uniquely named file and returns the path.
fn temp_file(name: &str, data: &[u8]) -> PathBuf {
    let path = std::env::temp_dir().join(format!("rpi-loader-test-{name}"));
    std::fs::write(&path, data).expect("writing the test fixture file");
    path
}

/// The device half of the link, driven over the pty's master side.
struct FakeDevice {
    /// The master end. Reads time out rather than blocking forever, so a
    /// CLI that never sends what the script expects fails the test
    /// instead of hanging it.
    port: File,
}

impl FakeDevice {
    /// Reads exactly `n` bytes, panicking on a timeout.
    fn read_exact(&mut self, n: usize) -> Vec<u8> {
        let mut buf = vec![0u8; n];
        let mut filled = 0;
        while filled < n {
            match self.port.read(&mut buf[filled..]) {
                Ok(0) => panic!("device timed out after {filled} of {n} expected bytes"),
                Ok(got) => filled += got,
                Err(e) => panic!("device read failed: {e}"),
            }
        }
        buf
    }

    /// Reads one byte.
    fn read_u8(&mut self) -> u8 {
        self.read_exact(1)[0]
    }

    /// Reads a little-endian `u32`.
    fn read_u32(&mut self) -> u32 {
        u32::from_le_bytes(self.read_exact(4).try_into().unwrap())
    }

    /// Writes every byte, flushing so the CLI sees it immediately.
    fn write_all(&mut self, data: &[u8]) {
        self.port.write_all(data).expect("device write");
        self.port.flush().expect("device flush");
    }

    /// Answers the host's HELLO, skipping anything before the magic the
    /// way the real device's framing does.
    fn handshake(&mut self) {
        let mut matched = 0;
        while matched < HELLO.len() {
            let b = self.read_u8();
            matched = if b == HELLO[matched] {
                matched + 1
            } else if b == HELLO[0] {
                1
            } else {
                0
            };
        }
        let mut reply = ACK.to_vec();
        reply.push(VERSION);
        self.write_all(&reply);
    }

    /// Reads a command byte and asserts which command it is.
    fn expect_command(&mut self, expected: u8) {
        let got = self.read_u8();
        assert_eq!(got, expected, "wrong command byte");
    }

    /// Reads a `SET_BAUD` command and acknowledges it, returning the rate
    /// the host asked for.
    fn expect_set_baud(&mut self) -> u32 {
        self.expect_command(CMD_SET_BAUD);
        let baud = self.read_u32();
        self.write_all(&[OK]);
        baud
    }

    /// Reads a path argument: u16 LE length then UTF-8 bytes.
    fn read_path(&mut self) -> String {
        let len = u16::from_le_bytes(self.read_exact(2).try_into().unwrap());
        String::from_utf8(self.read_exact(len as usize)).expect("path is UTF-8")
    }

    /// Receives host→device chunks, checking each CRC.
    ///
    /// With `reject_first`, one good chunk is rejected anyway, to drive
    /// the host's resend path.
    fn recv_chunks(&mut self, total: usize, chunk_size: usize, reject_first: bool) -> Vec<u8> {
        let mut out = Vec::with_capacity(total);
        let mut rejected = false;
        while out.len() < total {
            let this = chunk_size.min(total - out.len());
            let declared = self.read_u32();
            let data = self.read_exact(this);
            if reject_first && !rejected {
                rejected = true;
                self.write_all(&[FAIL]);
                continue;
            }
            if crc32(&data) == declared {
                out.extend_from_slice(&data);
                self.write_all(&[OK]);
            } else {
                self.write_all(&[FAIL]);
            }
        }
        out
    }

    /// Sends a device→host stream, resending a chunk the host rejects.
    fn send_bulk(&mut self, data: &[u8]) {
        let mut header = (data.len() as u32).to_le_bytes().to_vec();
        header.extend_from_slice(&(CHUNK as u32).to_le_bytes());
        self.write_all(&header);
        for piece in data.chunks(CHUNK) {
            let mut packet = crc32(piece).to_le_bytes().to_vec();
            packet.extend_from_slice(piece);
            loop {
                self.write_all(&packet);
                if self.read_u8() == OK {
                    break;
                }
            }
        }
    }
}

/// A running CLI attached to a fake device.
struct Fixture {
    /// The scripted device.
    device: FakeDevice,
    /// The CLI process, taken by [`Fixture::finish`].
    child: Option<Child>,
    /// The child's stdin, when it was piped.
    stdin: Option<ChildStdin>,
    /// Held open for the pty's whole life on purpose. Dropping it before
    /// the CLI opens the same path by name leaves the pty with no slave
    /// attached, and reads on the master then fail with EIO.
    _slave: OwnedFd,
}

impl Fixture {
    /// Spawns the CLI against a fresh pty, with stdin inherited.
    fn spawn(args: &[&str]) -> Self {
        Self::spawn_inner(args, false)
    }

    /// Spawns the CLI with its stdin piped, for the terminal tests.
    fn spawn_with_stdin(args: &[&str]) -> Self {
        Self::spawn_inner(args, true)
    }

    fn spawn_inner(args: &[&str], pipe_stdin: bool) -> Self {
        let pty = openpty(None, None).expect("openpty");

        // Raw on both ends: no echo, no line discipline rewriting bytes
        // in either direction. The device's own reads additionally get
        // VMIN=0/VTIME to bound them.
        for fd in [&pty.master, &pty.slave] {
            let mut attrs = tcgetattr(fd).expect("tcgetattr");
            cfmakeraw(&mut attrs);
            attrs.control_chars[SpecialCharacterIndices::VMIN as usize] = 0;
            attrs.control_chars[SpecialCharacterIndices::VTIME as usize] = 20; // 2s
            tcsetattr(fd, SetArg::TCSANOW, &attrs).expect("tcsetattr");
        }

        let device_path = ttyname(&pty.slave).expect("ttyname");
        let child = Command::new(env!("CARGO_BIN_EXE_rpi-loader"))
            .arg("--device")
            .arg(&device_path)
            .args(args)
            .stdin(if pipe_stdin {
                Stdio::piped()
            } else {
                Stdio::null()
            })
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .expect("spawning the CLI");

        let mut child = child;
        let stdin = child.stdin.take();
        Self {
            device: FakeDevice {
                port: File::from(pty.master),
            },
            child: Some(child),
            stdin,
            _slave: pty.slave,
        }
    }

    /// Writes to the child's stdin, as if typed.
    fn type_input(&mut self, data: &[u8]) {
        let stdin = self.stdin.as_mut().expect("stdin was not piped");
        stdin.write_all(data).expect("writing to the CLI's stdin");
        stdin.flush().expect("flushing the CLI's stdin");
    }

    /// Waits for the CLI to exit and collects its output.
    ///
    /// A watchdog kills the child if it outlives [`CHILD_TIMEOUT`], so a
    /// CLI that hangs fails the test rather than the test run.
    fn finish(mut self) -> Output {
        // Closing stdin lets a terminal session see end of input.
        drop(self.stdin.take());
        let child = self.child.take().expect("child already taken");
        let pid = Pid::from_raw(child.id() as i32);

        let done = Arc::new(AtomicBool::new(false));
        let watchdog_done = Arc::clone(&done);
        let watchdog = thread::spawn(move || {
            let deadline = std::time::Instant::now() + CHILD_TIMEOUT;
            while std::time::Instant::now() < deadline {
                if watchdog_done.load(Ordering::SeqCst) {
                    return;
                }
                thread::sleep(Duration::from_millis(50));
            }
            let _ = kill(pid, Signal::SIGKILL);
        });

        let output = child.wait_with_output().expect("waiting for the CLI");
        done.store(true, Ordering::SeqCst);
        let _ = watchdog.join();
        output
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        // Only reached when a test panicked before `finish`. Without this
        // the child would outlive the test holding the pty open.
        if let Some(child) = self.child.as_mut() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

/// Asserts the process succeeded, showing its stderr when it did not.
fn assert_success(output: &Output) {
    assert!(
        output.status.success(),
        "CLI failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn mem_write_negotiates_transfers_and_drops_back() {
    let data = payload();
    let file = temp_file("mem-write.bin", &data);
    let mut fx = Fixture::spawn(&["mem-write", "0x8000", file.to_str().unwrap()]);

    fx.device.handshake();
    assert_eq!(fx.device.expect_set_baud(), FAST_BAUD);

    fx.device.expect_command(CMD_MEM_WRITE);
    let total = fx.device.read_u32() as usize;
    let chunk = fx.device.read_u32() as usize;
    let addr = fx.device.read_u32();
    let declared_crc = fx.device.read_u32();
    assert_eq!((total, chunk, addr), (data.len(), CHUNK, 0x8000));
    assert_eq!(declared_crc, crc32(&data));
    fx.device.write_all(&[OK]);

    let received = fx.device.recv_chunks(total, chunk, false);
    assert_eq!(received, data, "device received a different payload");
    fx.device.write_all(&[OK]);

    // Back down, so the next invocation and any booted kernel find the
    // link at the rate they expect.
    assert_eq!(fx.device.expect_set_baud(), BASE_BAUD);
    assert_success(&fx.finish());
}

#[test]
fn mem_write_resends_a_rejected_chunk() {
    let data = payload();
    let file = temp_file("mem-write-retry.bin", &data);
    let mut fx = Fixture::spawn(&["mem-write", "0x8000", file.to_str().unwrap()]);

    fx.device.handshake();
    fx.device.expect_set_baud();
    fx.device.expect_command(CMD_MEM_WRITE);
    let total = fx.device.read_u32() as usize;
    let chunk = fx.device.read_u32() as usize;
    fx.device.read_u32();
    fx.device.read_u32();
    fx.device.write_all(&[OK]);

    let received = fx.device.recv_chunks(total, chunk, true);
    assert_eq!(received, data, "the resent chunk did not arrive intact");
    fx.device.write_all(&[OK]);
    fx.device.expect_set_baud();
    assert_success(&fx.finish());
}

#[test]
fn mem_write_at_base_baud_skips_negotiation() {
    let data = payload();
    let file = temp_file("mem-write-slow.bin", &data);
    let mut fx = Fixture::spawn(&[
        "mem-write",
        "0x8000",
        file.to_str().unwrap(),
        "--baud",
        "115200",
    ]);

    fx.device.handshake();
    // Straight to the command: there is nothing to negotiate when the
    // requested rate is the one already in use.
    fx.device.expect_command(CMD_MEM_WRITE);
    let total = fx.device.read_u32() as usize;
    let chunk = fx.device.read_u32() as usize;
    fx.device.read_u32();
    fx.device.read_u32();
    fx.device.write_all(&[OK]);
    fx.device.recv_chunks(total, chunk, false);
    fx.device.write_all(&[OK]);
    assert_success(&fx.finish());
}

#[test]
fn exec_sends_the_address() {
    let mut fx = Fixture::spawn(&["exec", "0x80000"]);
    fx.device.handshake();
    fx.device.expect_command(CMD_EXEC);
    assert_eq!(fx.device.read_u32(), 0x80000);
    fx.device.write_all(&[OK]);
    assert_success(&fx.finish());
}

#[test]
fn sd_list_prints_a_table() {
    let listing = "F\t131\tCONFIG.TXT\nD\t0\tOVERLAYS\nF\t78064\tKERNEL7.IMG\n";
    let mut fx = Fixture::spawn(&["sd-list", "/boot"]);

    fx.device.handshake();
    fx.device.expect_command(CMD_SD_LIST);
    assert_eq!(fx.device.read_path(), "/boot");
    fx.device.write_all(&[OK]);
    fx.device.send_bulk(listing.as_bytes());

    let output = fx.finish();
    assert_success(&output);
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("       78064  KERNEL7.IMG"),
        "sizes should be right-aligned: {stdout:?}"
    );
    assert!(
        stdout.contains("OVERLAYS/"),
        "directories should be marked: {stdout:?}"
    );
}

#[test]
fn sd_list_names_the_error_code() {
    let mut fx = Fixture::spawn(&["sd-list", "/nope"]);
    fx.device.handshake();
    fx.device.expect_command(CMD_SD_LIST);
    fx.device.read_path();
    fx.device.write_all(&[FAIL, ERR_NOT_FOUND]);

    let output = fx.finish();
    assert!(!output.status.success(), "a device FAIL must fail the CLI");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("no such file or directory"),
        "the error code should be named, not printed raw: {stderr:?}"
    );
}

#[test]
fn sd_read_writes_the_local_file() {
    let data = payload();
    let out_path = std::env::temp_dir().join("rpi-loader-test-sd-read.out");
    let _ = std::fs::remove_file(&out_path);
    let mut fx = Fixture::spawn(&["sd-read", "/BIG.BIN", out_path.to_str().unwrap()]);

    fx.device.handshake();
    fx.device.expect_set_baud();
    fx.device.expect_command(CMD_SD_READ);
    assert_eq!(fx.device.read_path(), "/BIG.BIN");
    fx.device.write_all(&[OK]);
    fx.device.send_bulk(&data);
    fx.device.expect_set_baud();

    assert_success(&fx.finish());
    assert_eq!(
        std::fs::read(&out_path).expect("the local file should exist"),
        data
    );
}

#[test]
fn sd_write_sends_the_payload() {
    let data = payload();
    let file = temp_file("sd-write.bin", &data);
    let mut fx = Fixture::spawn(&["sd-write", file.to_str().unwrap(), "/BIG.BIN"]);

    fx.device.handshake();
    fx.device.expect_set_baud();
    fx.device.expect_command(CMD_SD_WRITE);
    assert_eq!(fx.device.read_path(), "/BIG.BIN");
    let total = fx.device.read_u32() as usize;
    let chunk = fx.device.read_u32() as usize;
    fx.device.write_all(&[OK]);

    let received = fx.device.recv_chunks(total, chunk, false);
    assert_eq!(received, data);
    fx.device.write_all(&[OK]);
    fx.device.expect_set_baud();
    assert_success(&fx.finish());
}

#[test]
fn sd_write_names_a_failure_before_the_transfer() {
    let data = payload();
    let file = temp_file("sd-write-fail.bin", &data);
    let mut fx = Fixture::spawn(&["sd-write", file.to_str().unwrap(), "/BIG.BIN"]);

    fx.device.handshake();
    fx.device.expect_set_baud();
    fx.device.expect_command(CMD_SD_WRITE);
    fx.device.read_path();
    fx.device.read_u32();
    fx.device.read_u32();
    fx.device.write_all(&[FAIL, ERR_WRITE]);

    let output = fx.finish();
    assert!(!output.status.success());
    assert!(String::from_utf8_lossy(&output.stderr).contains("write failed"));
}

#[test]
fn sd_delete_and_mkdir_round_trip_their_paths() {
    for (args, command, path) in [
        (["sd-delete", "/BIG.BIN"], CMD_SD_DELETE, "/BIG.BIN"),
        (["sd-mkdir", "/LOGS"], CMD_SD_MKDIR, "/LOGS"),
    ] {
        let mut fx = Fixture::spawn(&args);
        fx.device.handshake();
        fx.device.expect_command(command);
        assert_eq!(fx.device.read_path(), path);
        fx.device.write_all(&[OK]);
        assert_success(&fx.finish());
    }
}

#[test]
fn terminal_carries_both_directions() {
    let mut fx = Fixture::spawn_with_stdin(&["terminal"]);

    // Device to host.
    fx.device.write_all(b"hello from device\r\n");
    // Host to device: typed input should arrive verbatim.
    fx.type_input(b"ls -l\r");
    assert_eq!(fx.device.read_exact(6), b"ls -l\r");

    // Ctrl-] leaves; it is not itself forwarded.
    fx.type_input(&[ESCAPE]);
    let output = fx.finish();
    assert_success(&output);
    assert!(
        String::from_utf8_lossy(&output.stdout).contains("hello from device"),
        "device output should reach stdout"
    );
}

#[test]
fn terminal_sends_what_precedes_the_escape() {
    let mut fx = Fixture::spawn_with_stdin(&["terminal"]);
    fx.type_input(b"abc");
    assert_eq!(fx.device.read_exact(3), b"abc");

    // Everything before the escape in the same read still goes out; only
    // the escape itself is swallowed.
    fx.type_input(&[b'd', b'e', b'f', ESCAPE, b'g']);
    assert_eq!(fx.device.read_exact(3), b"def");

    assert_success(&fx.finish());
}
