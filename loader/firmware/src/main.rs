#![no_std]
#![no_main]

extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write;
use embedded_alloc::LlffHeap as Heap;
use embedded_hal::i2c::I2c as _;
use resident_fat::{FatError, FileSystem};
use rpi_hal::i2c::I2c;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac::BSC0;
use rpi_hal::sd::{Sd, SdBlockDevice, SdBlockDeviceError};
use rpi_hal::timer::Timer;
use rpi_hal::{pac, uart::Uart};

// The boot stub is very nearly the only architecture-specific part of
// this loader: everything below (the command protocol, CRC, chunking,
// SD/FAT access) is shared, and the one other place that isn't is
// `exec`'s pair of barriers. See each file's module doc for the
// differences between them.
#[cfg(all(target_arch = "arm", not(armv6)))]
core::arch::global_asm!(include_str!("boot.s"));
#[cfg(armv6)]
core::arch::global_asm!(include_str!("boot6.s"));
#[cfg(target_arch = "aarch64")]
core::arch::global_asm!(include_str!("boot64.s"));

/// Marks the start of a session. Chosen to make accidental false
/// matches against line noise very unlikely (4 distinct bytes, no
/// repeated prefix/suffix, so the simple match-or-reset scan below is
/// correct without needing a full KMP table).
const HELLO: &[u8; 4] = b"RPIL";
/// HELLO reversed. A single response byte turned out to be too weak a
/// handshake in practice — a boot-time electrical transient can look
/// like a valid non-printable response byte, which was observed
/// causing a false-positive version match. A 4-byte ACK match is far
/// less likely to happen by accident.
const ACK: &[u8; 4] = b"LIPR";
/// What this loader answers a handshake with, so a host can tell which
/// commands it will understand. 2 adds the `CMD_EEPROM_*` pair to 1's
/// set.
///
/// Bumped for an addition, not just a change, because of how this loader
/// is used: it is flashed once and left there for months while the host
/// CLI is updated whenever. A newer CLI meeting an older loader is
/// therefore the *likely* mismatch, and without a version to compare it
/// shows up as an unknown command answered with `FAIL` — an error code
/// that says nothing about the real cause.
const PROTOCOL_VERSION: u8 = 2;
const OK: u8 = 1;
const FAIL: u8 = 0;

// The handshake and a booted kernel both come up at the base baud
// `Uart::init` selects (115200); a session may negotiate faster via
// `CMD_SET_BAUD` for the bulk transfers, and the host is responsible for
// dropping back to the base rate before `CMD_EXEC` so a loaded kernel's
// output lands where the host is already listening. The device never
// needs to name the base rate itself — it only ever switches on request.

// Command bytes. The host sends one of these after the version
// exchange; the device services it and returns to the command loop for
// the next one, except `CMD_EXEC`, which jumps and never returns.
//
// `CMD_MEM_WRITE` writes a checksummed blob to a memory address (the old
// kernel-upload path, minus the jump). `CMD_SET_BAUD` is followed by a
// `u32` LE baud; the device ACKs at the *current* baud, then switches.
// `CMD_EXEC` is followed by a `u32` LE address to jump to. The `CMD_SD_*`
// commands read/write/list/delete files and create directories on the SD
// card's FAT boot partition. The `CMD_EEPROM_*` commands read and write a
// serial EEPROM on the HAT ID bus (see [`init_i2c`]).
const CMD_MEM_WRITE: u8 = 1;
const CMD_SET_BAUD: u8 = 2;
const CMD_EXEC: u8 = 3;
const CMD_SD_LIST: u8 = 4;
const CMD_SD_READ: u8 = 5;
const CMD_SD_WRITE: u8 = 6;
const CMD_SD_DELETE: u8 = 7;
const CMD_SD_MKDIR: u8 = 8;
const CMD_EEPROM_READ: u8 = 9;
const CMD_EEPROM_WRITE: u8 = 10;

// Error codes, sent as the byte right after a leading `FAIL` when a
// command can't even begin (bad path, SD bring-up failed, filesystem
// error, etc.). The host prints a name for each; `ERR_FS` is what is left
// over once every failure a user can act on has a code of its own (see
// [`err_code`]).
//
// Codes are never reused. 4 meant "directory listing too large", back
// when a listing was built in a fixed buffer, and 6 "write failed", which
// the `sd-write` commit now reports by its real cause instead; neither is
// sent any more, but a host still has to name them for an older loader.
const ERR_SD_INIT: u8 = 1;
const ERR_NOT_FOUND: u8 = 2;
const ERR_FS: u8 = 3;
const ERR_BAD_PATH: u8 = 5;
const ERR_I2C: u8 = 7;
const ERR_VERIFY: u8 = 8;
const ERR_RANGE: u8 = 9;
const ERR_READBACK: u8 = 10;
const ERR_BAD_NAME: u8 = 11;
const ERR_EXISTS: u8 = 12;
const ERR_NOT_DIR: u8 = 13;
const ERR_IS_DIR: u8 = 14;
const ERR_NO_SPACE: u8 = 15;
const ERR_NO_MEMORY: u8 = 16;
const ERR_CARD: u8 = 17;
const ERR_NO_VOLUME: u8 = 18;

/// Stay clear of the relocated loader's own copy — see `boot.s`. Every
/// `CMD_MEM_WRITE`/`CMD_EXEC` address must fall below this, so a client
/// can never scribble on (or jump into) the running loader.
const RELOC_ADDR: usize = 0x0020_0000;

/// Chunk size for both directions of bulk transfer. The host is told
/// this value in every device→host stream header, and `CMD_MEM_WRITE`/
/// `CMD_SD_WRITE` reject a header whose chunk size exceeds it (that's
/// what bounds `CHUNK_BUF`).
const STREAM_CHUNK_SIZE: usize = 4096;

/// Longest path accepted from the host. Anything longer is drained off
/// the wire (to stay in sync) and rejected with [`ERR_BAD_PATH`].
const MAX_PATH: usize = 255;

/// BSC clock divider for the EEPROM bus: the reset default, 100kHz at a
/// 150MHz core clock and 166kHz at 250MHz. Either is within what every
/// 24C-series part does, and the HAT specification only asks for 100kHz —
/// there is nothing to gain from computing an exact rate here, which
/// would mean asking the mailbox for the real core clock first.
const EEPROM_CDIV: u16 = 0x05dc;

/// One past the highest EEPROM byte this loader will address. The two-byte
/// addressing it uses (what every part from the 24C32 up expects, and the
/// HAT specification's floor is a 24C32) reaches 64 KiB and no further, so
/// a request past this is refused with [`ERR_RANGE`] rather than
/// silently wrapping to the start of the device.
const EEPROM_LIMIT: usize = 0x1_0000;

/// Largest page write accepted from the host. A page write must not cross
/// the part's own page boundary — the address counter wraps within the
/// page rather than carrying, so an overrunning write silently overwrites
/// the *start* of the same page. The host names its part's page size; this
/// only bounds the buffer.
const EEPROM_MAX_PAGE: usize = 128;

/// How long to leave a page alone after writing it, while the part
/// performs its internal write cycle. Datasheets quote 5ms maximum for
/// this family; this is that, rounded up.
const EEPROM_WRITE_CYCLE_MS: u32 = 6;

/// How many times the verifying read is attempted before the page is
/// called a failure. More than one because [`EEPROM_WRITE_CYCLE_MS`] is a
/// datasheet number rather than a measurement of the part actually
/// fitted: a slow one answers nothing on the first attempt, and the cost
/// of finding out is one more transfer.
const EEPROM_READBACK_ATTEMPTS: u32 = 4;

/// The heap `resident-fat` keeps a mounted volume's allocation table and
/// directories in, and the `sd-*` commands their file contents.
///
/// Empty until [`init_heap`] gives it a region. Should that fail, the
/// `sd-*` commands refuse with [`ERR_NO_MEMORY`] before touching the card
/// (see [`mount`]) and everything else works as before.
#[global_allocator]
static HEAP: Heap = Heap::empty();

extern "C" {
    /// Top of the loader's stack, placed by `linker.ld`/`linker64.ld` above
    /// everything else the relocated loader occupies. Only its address is
    /// meaningful.
    static __stack_top: u8;
}

/// The `critical-section` implementation [`HEAP`]'s lock goes through.
///
/// Does nothing, and that is sound: the loader runs on one core and never
/// enables an interrupt, so there is no other context that could enter a
/// critical section concurrently. rpi-hal's own implementation, which
/// masks IRQs, is `rt`-gated, and this loader builds without `rt`.
struct SingleCore;
critical_section::set_impl!(SingleCore);

unsafe impl critical_section::Impl for SingleCore {
    unsafe fn acquire() -> critical_section::RawRestoreState {
        false
    }

    unsafe fn release(_: critical_section::RawRestoreState) {}
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    halt();
}

/// Loader entry point, reached from the relocated copy set up by
/// `boot.s`/`boot64.s`. Runs the host handshake, then a command loop
/// that never returns (the only exit is a client `CMD_EXEC` jumping
/// into a freshly loaded image).
#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    // The SD commands re-`steal()` the peripherals they need (EMMC, GPIO,
    // VCMAILBOX) and bring the card up fresh each time, so `kmain` holds
    // on to a `Timer` only — the one long-lived borrow `SdBlockDevice`
    // needs for its block transfers. Everything else in `peripherals` goes unused here.
    let timer = Timer::new(peripherals.SYSTMR);

    if let Err(e) = init_heap() {
        let _ = writeln!(uart, "rpi-loader: no heap ({e:?}), sd-* unavailable");
    }
    let _ = writeln!(uart, "rpi-loader: relocated, waiting for host");

    // Block waiting for the host to send HELLO — no retry/timeout
    // needed on this side, a plain blocking read already means the Pi
    // can be powered on before the host starts, or the host can start
    // first and wait; either order works.
    wait_for_hello(&mut uart);
    greet(&mut uart);

    // Command loop. Each command reads its own arguments, does its work,
    // and loops back for the next — the loader stays resident so the host
    // can drive any sequence of memory/SD operations over one session.
    // Only `CMD_EXEC` breaks out, by jumping into a loaded image.
    //
    // The loader outlives any single host invocation: each host CLI
    // subcommand is its own process that reconnects to an
    // already-running loader. So `read_command` also answers a fresh
    // HELLO by re-greeting — that's how the second and later invocations
    // handshake without the Pi being power-cycled.
    let mut chunk_buf = [0u8; STREAM_CHUNK_SIZE];
    loop {
        match read_command(&mut uart) {
            CMD_MEM_WRITE => cmd_mem_write(&mut uart, &mut chunk_buf),
            CMD_SET_BAUD => cmd_set_baud(&mut uart),
            CMD_EXEC => {
                let addr = read_u32_le(&mut uart) as usize;
                // A valid target sits below the loader's own relocated
                // copy (that's where `CMD_MEM_WRITE` loads images), so a
                // client can't ask us to jump into the running loader or
                // to a null address.
                if addr == 0 || addr >= RELOC_ADDR {
                    uart.write_byte(FAIL);
                } else {
                    uart.write_byte(OK);
                    // Print nothing after OK: the host may already be
                    // switching modes, and there's a kernel about to take
                    // the UART. Let the ACK fully drain, then jump.
                    uart.flush();
                    exec(addr);
                }
            }
            CMD_SD_LIST => {
                if let Err(code) = cmd_sd_list(&mut uart, &timer) {
                    uart.write_byte(FAIL);
                    uart.write_byte(code);
                }
            }
            CMD_SD_READ => {
                if let Err(code) = cmd_sd_read(&mut uart, &timer) {
                    uart.write_byte(FAIL);
                    uart.write_byte(code);
                }
            }
            CMD_SD_WRITE => {
                if let Err(code) = cmd_sd_write(&mut uart, &timer) {
                    uart.write_byte(FAIL);
                    uart.write_byte(code);
                }
            }
            CMD_SD_DELETE => {
                if let Err(code) = cmd_sd_delete(&mut uart, &timer) {
                    uart.write_byte(FAIL);
                    uart.write_byte(code);
                }
            }
            CMD_SD_MKDIR => {
                if let Err(code) = cmd_sd_mkdir(&mut uart, &timer) {
                    uart.write_byte(FAIL);
                    uart.write_byte(code);
                }
            }
            CMD_EEPROM_READ => {
                if let Err(code) = cmd_eeprom_read(&mut uart, &timer, &mut chunk_buf) {
                    uart.write_byte(FAIL);
                    uart.write_byte(code);
                }
            }
            CMD_EEPROM_WRITE => {
                if let Err(code) = cmd_eeprom_write(&mut uart, &timer, &mut chunk_buf) {
                    uart.write_byte(FAIL);
                    uart.write_byte(code);
                }
            }
            _ => uart.write_byte(FAIL),
        }
    }
}

/// `CMD_MEM_WRITE`: write a checksummed blob to a memory address.
///
/// Reads a 16-byte header (`total_size`, `chunk_size`, `load_addr`,
/// `overall_checksum`, all `u32` LE), validates it against the memory
/// map, receives the payload as CRC-checked chunks, then re-verifies the
/// whole thing against `overall_checksum` read back out of memory. This
/// is the old kernel-upload path with the jump split out into `CMD_EXEC`.
fn cmd_mem_write(uart: &mut Uart, chunk_buf: &mut [u8]) {
    let total_size = read_u32_le(uart) as usize;
    let chunk_size = read_u32_le(uart) as usize;
    let load_addr = read_u32_le(uart) as usize;
    let overall_checksum = read_u32_le(uart);

    let valid = total_size != 0
        && chunk_size != 0
        && chunk_size <= chunk_buf.len()
        && load_addr
            .checked_add(total_size)
            .is_some_and(|end| end <= RELOC_ADDR);
    if !valid {
        uart.write_byte(FAIL);
        return;
    }
    uart.write_byte(OK);

    // Each chunk is retried (host resends the same chunk_crc + data)
    // until its checksum matches, making the transfer self-healing
    // against whatever was corrupting/dropping bytes on long transfers.
    let dest = load_addr as *mut u8;
    let mut offset = 0;
    while offset < total_size {
        let this_len = core::cmp::min(chunk_size, total_size - offset);
        recv_chunk(uart, chunk_buf, this_len);
        for (i, &b) in chunk_buf[..this_len].iter().enumerate() {
            unsafe { core::ptr::write_volatile(dest.add(offset + i), b) };
        }
        // ACK only now that the chunk is stored and we're about to loop
        // back to `recv_chunk` — see that function on why the success
        // `OK` doubles as flow control.
        uart.write_byte(OK);
        offset += this_len;
    }

    // Recompute over what actually ended up in memory (not accumulated
    // during the chunk loop above) so this can't be fooled by any
    // bookkeeping mistake in that loop.
    let mut overall_crc = Crc32::new();
    for i in 0..total_size {
        overall_crc.update(unsafe { core::ptr::read_volatile(dest.add(i)) });
    }
    if overall_crc.finish() != overall_checksum {
        uart.write_byte(FAIL);
        return;
    }
    uart.write_byte(OK);
}

/// `CMD_SET_BAUD`: switch the link to a faster rate for the bulk
/// transfers that follow. Reads a `u32` LE baud; the reply must reach
/// the host at the *current* baud, so it ACKs first and lets that fully
/// drain (flush) before the divisor changes — otherwise the tail of the
/// ACK byte goes out at the new rate and the host misreads it.
/// `set_baud`'s own bool tells us whether the rate is representable; on a
/// no, the link is left untouched and the host stays put too.
fn cmd_set_baud(uart: &mut Uart) {
    let baud = read_u32_le(uart);
    if baud_representable(baud) {
        uart.write_byte(OK);
        uart.flush();
        let _ = uart.set_baud(baud);
    } else {
        uart.write_byte(FAIL);
    }
}

/// `CMD_SD_LIST`: list a directory on the FAT boot partition.
///
/// Reads a path (empty/`/` means the root), builds a line-based text
/// listing (`type\tsize\tname`, one entry per line) in RAM, then streams
/// it to the host. `Err(code)` here means the listing never started (bad
/// path, SD/FS error); the caller sends `FAIL` + `code`.
///
/// Names are long names wherever the volume has one, falling back to the
/// 8.3 name for an entry that has only that.
fn cmd_sd_list(uart: &mut Uart, timer: &Timer) -> Result<(), u8> {
    let mut path_buf = [0u8; MAX_PATH];
    let path = read_path(uart, &mut path_buf)?;

    let mut volume = mount(timer)?;
    // Built whole before the leading `OK`, so a directory that cannot be
    // read fails cleanly instead of mid-stream, and so the stream below can
    // resend a chunk without walking the directory again.
    let mut listing = String::new();
    for entry in volume.open_dir(path).map_err(err_code)?.iter() {
        let kind = if entry.is_directory() { 'D' } else { 'F' };
        let _ = writeln!(listing, "{}\t{}\t{}", kind, entry.len(), entry.name());
    }

    uart.write_byte(OK);
    send_bulk(uart, listing.as_bytes());
    Ok(())
}

/// `CMD_SD_READ`: stream a file off the FAT boot partition to the host.
///
/// Reads a path, reads the whole file into RAM, sends its length, then
/// streams the contents as CRC-checked chunks. `Err(code)` means the read
/// never started (bad path, not found, SD/FS error, or no room for the
/// file); the caller sends `FAIL` + `code`.
///
/// Whole rather than a chunk at a time because that is what makes a card
/// error a clean failure: the file is off the card before the leading
/// `OK`, so once streaming starts nothing can stop it short. It also
/// costs the card one transfer per contiguous run of the file rather than
/// one per chunk.
fn cmd_sd_read(uart: &mut Uart, timer: &Timer) -> Result<(), u8> {
    let mut path_buf = [0u8; MAX_PATH];
    let path = entry_path(read_path(uart, &mut path_buf)?)?;

    let mut volume = mount(timer)?;
    let file = volume.open(path).map_err(err_code)?;
    let data = volume.read_all(&file).map_err(err_code)?;

    uart.write_byte(OK);
    send_bulk(uart, &data);
    Ok(())
}

/// `CMD_SD_WRITE`: receive a file from the host and write it to the FAT
/// boot partition, creating or replacing it.
///
/// Reads a path, then a `u32` LE `total_size` and `u32` LE `chunk_size`,
/// receives the payload into RAM as CRC-checked chunks, then writes the
/// file in one go. `Err(code)` means the write never started; the caller
/// sends `FAIL` + `code`. After the leading `OK`, a final status byte
/// (`OK`, or `FAIL` + the code for why) reports the committed result.
///
/// Receiving the whole file before writing any of it has three payoffs.
/// The length is known when the file is created, so `resident-fat`
/// allocates its chain at once and the file lands contiguous, written in
/// one transfer per run. Each chunk's `OK` follows a copy into RAM rather
/// than an SD write, so the link is never idle waiting on the card. And a
/// transfer abandoned partway (the host killed, the cable pulled) leaves
/// the old file on the card untouched rather than truncated.
fn cmd_sd_write(uart: &mut Uart, timer: &Timer) -> Result<(), u8> {
    let mut path_buf = [0u8; MAX_PATH];
    let path = read_path(uart, &mut path_buf)?;
    let total_size = read_u32_le(uart) as usize;
    let chunk_size = read_u32_le(uart) as usize;
    let path = entry_path(path)?;
    if chunk_size == 0 || chunk_size > STREAM_CHUNK_SIZE {
        return Err(ERR_FS);
    }

    let mut volume = mount(timer)?;
    // Everything that can be known about the destination is checked
    // before the transfer, not after it: a missing parent directory, or a
    // directory where the file should go, would otherwise cost the whole
    // upload to find out.
    let parent = path.rsplit_once('/').map_or("", |(parent, _)| parent);
    volume.open_dir(parent).map_err(err_code)?;
    match volume.open(path) {
        Ok(_) | Err(resident_fat::Error::NotFound { .. }) => {}
        Err(e) => return Err(err_code(e)),
    }

    let mut data = Vec::new();
    data.try_reserve_exact(total_size)
        .map_err(|_| ERR_NO_MEMORY)?;
    data.resize(total_size, 0);

    uart.write_byte(OK);

    let mut offset = 0;
    while offset < total_size {
        let this_len = core::cmp::min(chunk_size, total_size - offset);
        recv_chunk(uart, &mut data[offset..], this_len);
        uart.write_byte(OK);
        offset += this_len;
    }

    // Synced even when the write failed: a failure partway may already
    // have marked the volume dirty, and leaving it so would have the next
    // mount (and every PC the card is put in) think it was pulled mid-write.
    let written = volume.write_file(path, &data).map(|_| ());
    let synced = volume.unmount().map(|_| ());
    match written.and(synced) {
        Ok(()) => uart.write_byte(OK),
        Err(e) => {
            uart.write_byte(FAIL);
            uart.write_byte(err_code(e));
        }
    }
    Ok(())
}

/// `CMD_SD_DELETE`: delete a file from the FAT boot partition.
///
/// Reads a path and deletes the file it names, long-name entries and all.
/// Replies `OK` on success; `Err(code)` (bad path, not found, SD/FS error)
/// means the caller sends `FAIL` + `code`. Deletes files only — a
/// directory is [`ERR_IS_DIR`].
fn cmd_sd_delete(uart: &mut Uart, timer: &Timer) -> Result<(), u8> {
    let mut path_buf = [0u8; MAX_PATH];
    let path = entry_path(read_path(uart, &mut path_buf)?)?;

    let mut volume = mount(timer)?;
    let removed = volume.remove(path);
    let synced = volume.unmount().map(|_| ());
    removed.and(synced).map_err(err_code)?;

    uart.write_byte(OK);
    Ok(())
}

/// `CMD_SD_MKDIR`: create a directory on the FAT boot partition.
///
/// Reads a path and creates the directory it names. Replies `OK` on
/// success; `Err(code)` (bad path, SD/FS error) means the caller sends
/// `FAIL` + `code`. Only the final component is created — the parent
/// directories must already exist, so this is a single `mkdir`, not
/// `mkdir -p`. Creating a directory that already exists is
/// [`ERR_EXISTS`].
fn cmd_sd_mkdir(uart: &mut Uart, timer: &Timer) -> Result<(), u8> {
    let mut path_buf = [0u8; MAX_PATH];
    let path = entry_path(read_path(uart, &mut path_buf)?)?;

    let mut volume = mount(timer)?;
    let created = volume.create_dir(path).map(|_| ());
    let synced = volume.unmount().map(|_| ());
    created.and(synced).map_err(err_code)?;

    uart.write_byte(OK);
    Ok(())
}

/// `CMD_EEPROM_READ`: stream bytes out of a serial EEPROM on the HAT ID
/// bus to the host.
///
/// Reads a `u8` device address, a `u32` LE `offset` and a `u32` LE
/// `length`, probes the device, then streams the range as CRC-checked
/// chunks. `Err(code)` means the read never started — an out-of-range
/// request ([`ERR_RANGE`]) or nothing answering at that address
/// ([`ERR_I2C`]); the caller sends `FAIL` + `code`.
///
/// The probe is what makes "nothing is fitted" a clean failure: once the
/// leading `OK` is out, the stream is committed, and a later bus error
/// can only stop it short and leave the host's per-chunk timeout to
/// report it.
fn cmd_eeprom_read(uart: &mut Uart, timer: &Timer, chunk_buf: &mut [u8]) -> Result<(), u8> {
    let address = uart.read_byte();
    let offset = read_u32_le(uart) as usize;
    let length = read_u32_le(uart) as usize;
    if length == 0 || !in_eeprom_range(offset, length) {
        return Err(ERR_RANGE);
    }

    let mut i2c = init_i2c(timer);
    let mut probe = [0u8; 1];
    eeprom_read_at(&mut i2c, address, offset as u16, &mut probe).map_err(|_| ERR_I2C)?;

    uart.write_byte(OK);
    write_u32(uart, length as u32);
    write_u32(uart, STREAM_CHUNK_SIZE as u32);

    let mut done = 0;
    while done < length {
        let want = core::cmp::min(STREAM_CHUNK_SIZE, length - done);
        let at = (offset + done) as u16;
        if eeprom_read_at(&mut i2c, address, at, &mut chunk_buf[..want]).is_err() {
            break;
        }
        send_chunk(uart, &chunk_buf[..want]);
        done += want;
    }
    Ok(())
}

/// `CMD_EEPROM_WRITE`: receive an image from the host and program it into
/// a serial EEPROM on the HAT ID bus.
///
/// Reads a `u8` device address, then `u32` LE `offset`, `total_size`,
/// `chunk_size` and `page_size`, and receives the payload as CRC-checked
/// chunks, programming each chunk a page at a time. `Err(code)` means the
/// write never started; the caller sends `FAIL` + `code`. After the
/// leading `OK`, chunks are always drained to keep the link in sync even
/// once programming has failed, and a final status byte (`OK`, or `FAIL` +
/// [`ERR_I2C`]/[`ERR_READBACK`]/[`ERR_VERIFY`], which say respectively
/// that the page write was not acknowledged, that the part never answered
/// the read that follows it, and that it answered with something other
/// than what was written) reports the committed result. A header that
/// asks for something outside the device's reach — past [`EEPROM_LIMIT`],
/// or a page larger than [`EEPROM_MAX_PAGE`] — is [`ERR_RANGE`], refused
/// before any of it is written.
///
/// `page_size` comes from the host because the device cannot know what
/// part is fitted, and a page write that crosses the part's page boundary
/// wraps to the start of that page instead of carrying — corrupting data
/// already written rather than failing. It must divide the page size of
/// the real part; the HAT specification's floor (a 24C32) has 32-byte
/// pages, which is what the host defaults to.
fn cmd_eeprom_write(uart: &mut Uart, timer: &Timer, chunk_buf: &mut [u8]) -> Result<(), u8> {
    let address = uart.read_byte();
    let offset = read_u32_le(uart) as usize;
    let total_size = read_u32_le(uart) as usize;
    let chunk_size = read_u32_le(uart) as usize;
    let page_size = read_u32_le(uart) as usize;

    if total_size == 0 || !in_eeprom_range(offset, total_size) {
        return Err(ERR_RANGE);
    }
    // A non-power-of-two page size would make the "distance to the next
    // page boundary" arithmetic below wrong, and no part in this family
    // has one.
    if chunk_size == 0
        || chunk_size > chunk_buf.len()
        || page_size == 0
        || page_size > EEPROM_MAX_PAGE
        || !page_size.is_power_of_two()
    {
        return Err(ERR_RANGE);
    }

    let mut i2c = init_i2c(timer);
    uart.write_byte(OK);

    let mut failure = None;
    let mut done = 0;
    while done < total_size {
        let this_len = core::cmp::min(chunk_size, total_size - done);
        recv_chunk(uart, chunk_buf, this_len);
        if failure.is_none() {
            failure = eeprom_write_pages(
                &mut i2c,
                timer,
                address,
                offset + done,
                &chunk_buf[..this_len],
                page_size,
            )
            .err();
        }
        // ACK after programming, not before — the same flow control the SD
        // write path relies on (see `recv_chunk`), and this side is far
        // slower: a page's internal write cycle is milliseconds, during
        // which nothing is draining the RX FIFO.
        uart.write_byte(OK);
        done += this_len;
    }

    match failure {
        None => uart.write_byte(OK),
        Some(code) => {
            uart.write_byte(FAIL);
            uart.write_byte(code);
        }
    }
    Ok(())
}

/// Programs `data` into the EEPROM starting at `at`, one page write at a
/// time, waiting out each internal write cycle and reading the page back
/// to confirm it took.
///
/// The read-back is not belt and braces: a write-protected part (the `WP`
/// pin tied high, which on a board with the HAT ID EEPROM's write protect
/// on a jumper is the normal state) acknowledges every byte and stores
/// none. Without the verify this command would report a clean success and
/// leave the EEPROM exactly as it was.
///
/// The wait between the two is a fixed delay rather than the acknowledge
/// polling the datasheet also describes. Polling is the faster technique
/// — a part is typically ready in well under its specified time — but it
/// has to begin after the part has registered the STOP that starts the
/// write cycle, and two back-to-back BSC transactions are only tens of
/// microseconds apart. Polled that early it reported ready when it was
/// not: the first page of an image landed and the sequence then failed on
/// the transfer after it, leaving the rest of the EEPROM erased. Waiting
/// out [`EEPROM_WRITE_CYCLE_MS`] has no such race, and the read-back that
/// follows is the real evidence the page took — so the delay only has to
/// be long enough, not exact.
fn eeprom_write_pages(
    i2c: &mut I2c<'_, BSC0>,
    timer: &Timer,
    address: u8,
    at: usize,
    data: &[u8],
    page_size: usize,
) -> Result<(), u8> {
    // Two bytes of address ahead of the payload, since a page write is one
    // I2C transaction: address high, address low, then the page's bytes.
    let mut packet = [0u8; 2 + EEPROM_MAX_PAGE];
    let mut readback = [0u8; EEPROM_MAX_PAGE];

    let mut written = 0;
    while written < data.len() {
        let at = at + written;
        // Stop at the next page boundary: `at` is not necessarily aligned
        // (an `--offset` can start anywhere), so the first write of a run
        // is usually short and the rest are full pages.
        let to_boundary = page_size - (at % page_size);
        let len = core::cmp::min(to_boundary, data.len() - written);

        packet[..2].copy_from_slice(&(at as u16).to_be_bytes());
        packet[2..2 + len].copy_from_slice(&data[written..written + len]);
        i2c.write(address, &packet[..2 + len])
            .map_err(|_| ERR_I2C)?;

        let mut read = Err(());
        for _ in 0..EEPROM_READBACK_ATTEMPTS {
            timer.delay_ms(EEPROM_WRITE_CYCLE_MS);
            read = eeprom_read_at(i2c, address, at as u16, &mut readback[..len]).map_err(|_| ());
            if read.is_ok() {
                break;
            }
        }
        // A read that never answered says something different from one
        // that answered wrongly: the first is a part still busy (or gone),
        // the second is a write that did not take.
        read.map_err(|_| ERR_READBACK)?;
        if readback[..len] != data[written..written + len] {
            return Err(ERR_VERIFY);
        }
        written += len;
    }
    Ok(())
}

/// Reads `buf.len()` bytes from `at`: a two-byte address write, then a
/// read the part answers from its address counter, incrementing through.
///
/// The two are separate transactions with a STOP between them rather than
/// a repeated start, which this hardware's driver does not offer. That is
/// safe here specifically because the address write carries no data byte:
/// the part latches the counter without starting a write cycle, and the
/// counter survives the STOP.
fn eeprom_read_at(
    i2c: &mut I2c<'_, BSC0>,
    address: u8,
    at: u16,
    buf: &mut [u8],
) -> Result<(), rpi_hal::i2c::Error> {
    i2c.write(address, &at.to_be_bytes())?;
    i2c.read(address, buf)
}

/// Whether `offset..offset + length` fits inside what two-byte addressing
/// can reach (see [`EEPROM_LIMIT`]).
fn in_eeprom_range(offset: usize, length: usize) -> bool {
    offset
        .checked_add(length)
        .is_some_and(|end| end <= EEPROM_LIMIT)
}

/// Brings BSC0 up on the HAT ID bus — GPIO0/1 (`ID_SD`/`ID_SC`), header
/// pins 27/28 — for the `eeprom-*` commands.
///
/// That is the bus a board's identity EEPROM sits on, and it is otherwise
/// idle once the firmware has read it during boot. Note which routing this
/// takes: BSC0's other one is GPIO44/45, the camera/display connector bus,
/// and the two cannot both be muxed at once.
///
/// Re-`steal()`s its peripherals per command, exactly as [`mount`] does,
/// so nothing has to be threaded through the command loop; sound for the
/// same reason — single core, and the previous command's driver is long
/// dropped.
fn init_i2c(timer: &Timer) -> I2c<'_, BSC0> {
    let peripherals = unsafe { pac::Peripherals::steal() };
    I2c::<BSC0>::init_id(&peripherals.GPIO, peripherals.BSC0, EEPROM_CDIV, timer)
}

/// A mounted FAT volume on the SD card.
type Volume<'t> = FileSystem<SdBlockDevice<'t>>;

/// What any operation on a [`Volume`] can fail with.
type FsError = resident_fat::Error<SdBlockDeviceError>;

/// Brings the SD card up from scratch and mounts its FAT volume.
///
/// Each SD command calls this fresh: it re-`steal()`s the peripherals it
/// needs, re-runs card identification and re-reads the allocation table,
/// trading some latency for a stateless design with no volume to thread
/// across the command loop — and with nothing held between commands, a
/// board reset between them can lose nothing. Stealing again is sound
/// here because the previous command's `Sd` has already been dropped —
/// there's never more than one live at a time on this single core. Maps
/// any bring-up failure to [`ERR_SD_INIT`].
///
/// The volume is the first FAT partition in the card's partition table,
/// whichever slot it is in, or the whole card when it has no table at all
/// (one formatted as a bare volume).
fn mount(timer: &Timer) -> Result<Volume<'_>, u8> {
    // Checked up front because not every allocation `resident-fat` makes
    // is fallible, and one that is not would halt the loader.
    if HEAP.free() == 0 {
        return Err(ERR_NO_MEMORY);
    }
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    // `Sd::steal_emmc` picks the right controller for the active chip --
    // the classic `EMMC` peripheral, or BCM2711's `Emmc2` (not part of
    // `Peripherals` at all, since it isn't in the PAC) -- see rpi-hal's
    // `sd.rs` "BCM2711" doc section.
    let emmc = unsafe { Sd::steal_emmc() };
    let sd = Sd::init(&peripherals.GPIO, emmc, &mut mailbox, timer).map_err(|_| ERR_SD_INIT)?;
    FileSystem::mount_first_fat(SdBlockDevice::new(sd, timer)).map_err(err_code)
}

/// Gives [`HEAP`] everything from the top of the loader's stack to the top
/// of the ARM's share of RAM.
///
/// That region is already out of every client's reach, which is what
/// makes it safe to hold a volume's allocation table in: `CMD_MEM_WRITE`
/// refuses anything ending above [`RELOC_ADDR`], and the loader itself
/// sits between that and `__stack_top`. So an image being loaded cannot
/// overwrite the heap, nor the heap the image.
///
/// Not `rpi_hal::mem::heap_region`, which starts the heap at `__bss_end`.
/// That is right for rpi-hal's own linker script, where the stacks sit
/// below `.bss`, and wrong for this loader's, where the stack is above it:
/// the heap would start inside the stack.
///
/// The top comes from the firmware rather than a constant because
/// `gpu_mem` in `config.txt` moves it.
fn init_heap() -> Result<(), rpi_hal::mailbox::Error> {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    let region = mailbox.arm_memory()?;
    // 8, not the word alignment the linker script promises: an allocator
    // hands out blocks aligned for `u64`, which is 8 on AArch32 too.
    let start = (&raw const __stack_top as usize).next_multiple_of(8);
    let end = region.base_address as usize + region.size_bytes as usize;
    if end > start {
        // SAFETY: called once, before anything allocates, on a region that
        // nothing else in the loader uses and no command can write to.
        unsafe { HEAP.init(start, end - start) };
    }
    Ok(())
}

/// Jumps to a freshly loaded image at `addr`, never returning.
///
/// We just wrote that image as data; without a barrier the core isn't
/// guaranteed to fetch those bytes as instructions when we jump — `dsb`
/// waits for the writes to complete, `isb` flushes the pipeline so the
/// next fetch actually sees them.
///
/// No instruction-cache maintenance is needed beyond that: this loader
/// runs with caches disabled throughout and is only ever entered fresh
/// from reset (you power-cycle to upload again — nothing jumps back into
/// it), so there are never stale cache lines over the load address for
/// these barriers not to cover.
///
/// AArch32's bare `dsb` defaults to the full-system domain; AArch64
/// requires the domain operand be spelled out (`dsb sy`). `isb` is
/// identical in both. ARMv6 has neither mnemonic and reaches the same
/// two barriers through CP15 instead -- `c7, c10, 4` and `c7, c5, 4`,
/// with a register operand that is ignored.
fn exec(addr: usize) -> ! {
    #[cfg(all(target_arch = "arm", not(armv6)))]
    unsafe {
        core::arch::asm!("dsb", "isb")
    };
    #[cfg(armv6)]
    unsafe {
        core::arch::asm!(
            "mcr p15, 0, {0}, c7, c10, 4",
            "mcr p15, 0, {0}, c7, c5, 4",
            in(reg) 0u32,
        )
    };
    #[cfg(target_arch = "aarch64")]
    unsafe {
        core::arch::asm!("dsb sy", "isb")
    };
    let entry: extern "C" fn() -> ! = unsafe { core::mem::transmute(addr) };
    entry()
}

/// Blocks scanning the input for the 4-byte [`HELLO`] magic.
///
/// The match-or-reset scan is correct without a full KMP table because
/// `HELLO`'s bytes are distinct with no repeated prefix (see its doc).
/// Used for the initial power-on handshake, where a boot-time electrical
/// transient can inject stray bytes before the host is even connected —
/// so this tolerates arbitrary leading noise rather than trusting the
/// first byte.
fn wait_for_hello(uart: &mut Uart) {
    let mut matched = 0;
    while matched < HELLO.len() {
        let byte = uart.read_byte();
        if byte == HELLO[matched] {
            matched += 1;
        } else if byte == HELLO[0] {
            matched = 1;
        } else {
            matched = 0;
        }
    }
}

/// Answers a completed handshake: the [`ACK`] magic followed by the
/// protocol version byte.
fn greet(uart: &mut Uart) {
    for &b in ACK {
        uart.write_byte(b);
    }
    uart.write_byte(PROTOCOL_VERSION);
}

/// Reads the next command byte, transparently re-greeting on a fresh
/// HELLO so a newly-launched host tool can reconnect to the running
/// loader (see the command loop).
///
/// A byte equal to `HELLO[0]` is treated as the start of a reconnect: the
/// remaining three magic bytes are matched and, on success, the loader
/// re-greets and waits for the real command. `HELLO[0]` (`'R'`) is not a
/// valid command byte and the host never sends a bare one, so consuming
/// those bytes can't swallow a genuine command; anything else is returned
/// as the command for the loop to dispatch.
fn read_command(uart: &mut Uart) -> u8 {
    loop {
        let byte = uart.read_byte();
        if byte != HELLO[0] {
            return byte;
        }
        let mut matched = 1;
        while matched < HELLO.len() && uart.read_byte() == HELLO[matched] {
            matched += 1;
        }
        if matched == HELLO.len() {
            greet(uart);
        }
    }
}

/// Reads a length-prefixed path (`u16` LE length, then that many UTF-8
/// bytes) into `buf`, returning it as a `&str`.
///
/// Always consumes exactly `length` bytes off the wire, even when
/// rejecting — an over-length or non-UTF-8 path would otherwise leave
/// the link out of sync for the next command. Returns [`ERR_BAD_PATH`]
/// in those cases.
fn read_path<'a>(uart: &mut Uart, buf: &'a mut [u8; MAX_PATH]) -> Result<&'a str, u8> {
    let length = read_u16_le(uart) as usize;
    for i in 0..length {
        let b = uart.read_byte();
        if i < buf.len() {
            buf[i] = b;
        }
    }
    if length > buf.len() {
        return Err(ERR_BAD_PATH);
    }
    core::str::from_utf8(&buf[..length]).map_err(|_| ERR_BAD_PATH)
}

/// Trims the slashes at either end of a path that must name a file or
/// directory, refusing one that names nothing.
///
/// `resident-fat` already treats a leading slash as optional, but reads a
/// trailing one as an empty final component and refuses it as a bad name
/// — where `sd-mkdir /logs/` meaning `/logs` is what anyone typing it
/// expects. An empty path or bare `/` names the root, which no file or new
/// directory can be, and is [`ERR_BAD_PATH`].
fn entry_path(path: &str) -> Result<&str, u8> {
    let trimmed = path.trim_matches('/');
    if trimmed.is_empty() {
        Err(ERR_BAD_PATH)
    } else {
        Ok(trimmed)
    }
}

/// Maps a `resident-fat` error to the code sent to the host.
///
/// Every failure a user can do something about has its own code — a name
/// FAT cannot store, a path that is missing or the wrong kind, a full
/// card, a card that stopped answering, no FAT volume at all. What is left
/// ([`ERR_FS`]) is a volume `resident-fat` found inconsistent, which is a
/// job for `fsck` rather than for anything the host could retry.
fn err_code(e: FsError) -> u8 {
    use resident_fat::Error;
    match e {
        Error::NotFound { .. } => ERR_NOT_FOUND,
        Error::BadName { .. } => ERR_BAD_NAME,
        Error::AlreadyExists { .. } => ERR_EXISTS,
        Error::NotADirectory { .. } => ERR_NOT_DIR,
        Error::IsADirectory { .. } => ERR_IS_DIR,
        Error::DirectoryFull | Error::Fat(FatError::DiskFull { .. }) => ERR_NO_SPACE,
        Error::OutOfMemory { .. } => ERR_NO_MEMORY,
        Error::Device(_) => ERR_CARD,
        Error::Boot(_)
        | Error::NoPartitionTable
        | Error::NoSuchPartition { .. }
        | Error::NoFatPartition => ERR_NO_VOLUME,
        _ => ERR_FS,
    }
}

/// Receives one chunk into `buf[..len]`: reads a `u32` LE CRC then `len`
/// data bytes, retrying (the host resends on `FAIL`) until the CRC
/// matches, then returns with the validated bytes in `buf[..len]`.
///
/// Sends a `FAIL` for each bad attempt, but does *not* send the success
/// `OK` — the caller does that once it has finished processing the chunk
/// and is ready to receive the next one. That deferral is deliberate: the
/// host sends the next chunk the instant it sees the `OK`, streaming
/// ~4 KB back-to-back with no hardware flow control, and the PL011's
/// 16-byte RX FIFO overflows within ~85µs at 1.5Mbaud. If the caller ACKs
/// before a slow step (programming an EEPROM page, whose internal write
/// cycle is milliseconds), those incoming bytes are dropped and the
/// framing desyncs permanently. ACKing only once the caller is back at `recv_chunk` turns
/// the per-chunk `OK` into flow control, keeping the transfer lockstep.
fn recv_chunk(uart: &mut Uart, buf: &mut [u8], len: usize) {
    loop {
        let declared_crc = read_u32_le(uart);
        let mut crc = Crc32::new();
        for b in buf.iter_mut().take(len) {
            *b = uart.read_byte();
            crc.update(*b);
        }
        if crc.finish() == declared_crc {
            return;
        }
        uart.write_byte(FAIL);
    }
}

/// Sends one chunk to the host: a `u32` LE CRC then the bytes, resending
/// until the host ACKs `OK`. The mirror of [`recv_chunk`], and equally
/// self-healing — the host replies `FAIL` on a CRC mismatch and we resend
/// the same buffer.
fn send_chunk(uart: &mut Uart, buf: &[u8]) {
    let mut crc = Crc32::new();
    for &b in buf {
        crc.update(b);
    }
    let crc = crc.finish();
    loop {
        write_u32(uart, crc);
        for &b in buf {
            uart.write_byte(b);
        }
        if uart.read_byte() == OK {
            return;
        }
    }
}

/// Streams an in-RAM blob to the host: a `u32` LE total length and `u32`
/// LE chunk size, then the data as [`send_chunk`] chunks. Used by
/// `CMD_SD_LIST` and `CMD_SD_READ` after their leading `OK`.
fn send_bulk(uart: &mut Uart, data: &[u8]) {
    write_u32(uart, data.len() as u32);
    write_u32(uart, STREAM_CHUNK_SIZE as u32);
    for chunk in data.chunks(STREAM_CHUNK_SIZE) {
        send_chunk(uart, chunk);
    }
}

/// Whether [`Uart::set_baud`] would accept `baud`, checked *before*
/// switching so a `CMD_SET_BAUD` can be ACKed at the old baud and only
/// then applied (see [`cmd_set_baud`]). Mirrors the divisor math in
/// `rpi-hal`'s `Uart::set_baud` for the same 48MHz reference clock — the
/// two crates are separate repos, so this restates the check rather than
/// sharing it. `set_baud`'s return value is still the source of truth
/// for whether the switch actually happened.
fn baud_representable(baud: u32) -> bool {
    if baud == 0 {
        return false;
    }
    let ibrd = (192_000_000 + baud / 2) / baud / 64;
    ibrd != 0 && ibrd <= u16::MAX as u32
}

fn read_u16_le(uart: &mut Uart) -> u16 {
    let mut bytes = [0u8; 2];
    for b in bytes.iter_mut() {
        *b = uart.read_byte();
    }
    u16::from_le_bytes(bytes)
}

fn read_u32_le(uart: &mut Uart) -> u32 {
    let mut bytes = [0u8; 4];
    for b in bytes.iter_mut() {
        *b = uart.read_byte();
    }
    u32::from_le_bytes(bytes)
}

fn write_u32(uart: &mut Uart, value: u32) {
    for &b in &value.to_le_bytes() {
        uart.write_byte(b);
    }
}

fn halt() -> ! {
    loop {
        unsafe { core::arch::asm!("wfe") };
    }
}

/// CRC-32/ISO-HDLC (the same algorithm as `zlib.crc32` / gzip / PNG /
/// Ethernet FCS) — deliberately the standard one, not a custom checksum,
/// so the host side can compute it with Python's built-in `zlib.crc32`
/// and the two are guaranteed to agree by construction.
struct Crc32(u32);

impl Crc32 {
    fn new() -> Self {
        Self(0xFFFF_FFFF)
    }

    fn update(&mut self, byte: u8) {
        self.0 ^= byte as u32;
        for _ in 0..8 {
            if self.0 & 1 != 0 {
                self.0 = (self.0 >> 1) ^ 0xEDB8_8320;
            } else {
                self.0 >>= 1;
            }
        }
    }

    fn finish(&self) -> u32 {
        !self.0
    }
}
