#![no_std]
#![no_main]

use core::fmt::Write;
use embedded_sdmmc::{Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::sd::{Sd, SdCard, SdCardError};
use rpi_hal::timer::Timer;
use rpi_hal::{pac, uart::Uart};

// The boot stub is the only architecture-specific part of this loader:
// everything below (the command protocol, CRC, chunking, SD/FAT access)
// is shared. See each file's module doc for the AArch32/AArch64
// differences.
#[cfg(target_arch = "arm")]
core::arch::global_asm!(include_str!("boot.s"));
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
const PROTOCOL_VERSION: u8 = 1;
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
// card's FAT boot partition.
const CMD_MEM_WRITE: u8 = 1;
const CMD_SET_BAUD: u8 = 2;
const CMD_EXEC: u8 = 3;
const CMD_SD_LIST: u8 = 4;
const CMD_SD_READ: u8 = 5;
const CMD_SD_WRITE: u8 = 6;
const CMD_SD_DELETE: u8 = 7;
const CMD_SD_MKDIR: u8 = 8;

// Error codes, sent as the byte right after a leading `FAIL` when a
// command can't even begin (bad path, SD bring-up failed, filesystem
// error, etc.). A coarse classification is enough for the host to print
// something useful; the exact `embedded-sdmmc` variant isn't wired over.
const ERR_SD_INIT: u8 = 1;
const ERR_NOT_FOUND: u8 = 2;
const ERR_FS: u8 = 3;
const ERR_TOO_LARGE: u8 = 4;
const ERR_BAD_PATH: u8 = 5;
const ERR_WRITE: u8 = 6;

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

/// Cap on a single directory listing. Listings are built in RAM before
/// streaming, so an unusually large directory is rejected with
/// [`ERR_TOO_LARGE`] rather than overrunning the buffer.
const LISTING_CAP: usize = 8192;

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
    // on to a `Timer` only — the one long-lived borrow `SdCard` needs for
    // its block reads. Everything else in `peripherals` goes unused here.
    let timer = Timer::new(peripherals.SYSTMR);

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
                if let Err(code) = cmd_sd_read(&mut uart, &timer, &mut chunk_buf) {
                    uart.write_byte(FAIL);
                    uart.write_byte(code);
                }
            }
            CMD_SD_WRITE => {
                if let Err(code) = cmd_sd_write(&mut uart, &timer, &mut chunk_buf) {
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
/// listing (`type\tsize\tname`, one entry per line) into a bounded RAM
/// buffer, then streams it to the host. `Err(code)` here means the
/// listing never started (bad path, SD/FS error, or too large); the
/// caller sends `FAIL` + `code`.
fn cmd_sd_list(uart: &mut Uart, timer: &Timer) -> Result<(), u8> {
    let mut path_buf = [0u8; MAX_PATH];
    let path = read_path(uart, &mut path_buf)?;

    let vm = init_volume_mgr(timer)?;
    let volume = vm.open_volume(VolumeIdx(0)).map_err(err_code)?;
    let mut dir = volume.open_root_dir().map_err(err_code)?;
    for comp in path.split('/').filter(|c| !c.is_empty()) {
        dir.change_dir(comp).map_err(err_code)?;
    }

    // Build the listing in RAM first: `iterate_dir` is a single forward
    // pass with no rewind, so buffering here is what lets the stream
    // below retry a chunk without re-walking the directory.
    let mut listing = [0u8; LISTING_CAP];
    let mut w = ListingWriter::new(&mut listing);
    dir.iterate_dir(|entry| {
        let kind = if entry.attributes.is_directory() {
            'D'
        } else {
            'F'
        };
        let _ = writeln!(w, "{}\t{}\t{}", kind, entry.size, entry.name);
    })
    .map_err(err_code)?;
    if w.overflowed {
        return Err(ERR_TOO_LARGE);
    }
    let len = w.len;

    uart.write_byte(OK);
    send_bulk(uart, &listing[..len]);
    Ok(())
}

/// `CMD_SD_READ`: stream a file off the FAT boot partition to the host.
///
/// Reads a path, opens the file, sends its length, then streams the
/// contents as CRC-checked chunks. `Err(code)` means the read never
/// started (bad path, not found, SD/FS error); the caller sends `FAIL` +
/// `code`. Once the leading `OK` is sent, streaming is committed — a rare
/// mid-stream block-read error just stops early, and the host's
/// per-chunk timeout surfaces the short transfer.
fn cmd_sd_read(uart: &mut Uart, timer: &Timer, chunk_buf: &mut [u8]) -> Result<(), u8> {
    let mut path_buf = [0u8; MAX_PATH];
    let path = read_path(uart, &mut path_buf)?;
    let (dir_path, file_name) = split_parent(path)?;

    let vm = init_volume_mgr(timer)?;
    let volume = vm.open_volume(VolumeIdx(0)).map_err(err_code)?;
    let mut dir = volume.open_root_dir().map_err(err_code)?;
    for comp in dir_path.split('/').filter(|c| !c.is_empty()) {
        dir.change_dir(comp).map_err(err_code)?;
    }
    let file = dir
        .open_file_in_dir(file_name, Mode::ReadOnly)
        .map_err(err_code)?;
    let length = file.length();

    uart.write_byte(OK);
    write_u32(uart, length);
    write_u32(uart, STREAM_CHUNK_SIZE as u32);

    let mut remaining = length as usize;
    while remaining > 0 {
        let want = core::cmp::min(STREAM_CHUNK_SIZE, remaining);
        // `read` may return short (it reads block-aligned under the hood),
        // so fill the chunk before sending it. A `0`/`Err` means the file
        // ended sooner than `length()` claimed or a block read failed;
        // either way, send what we have and stop.
        let mut got = 0;
        while got < want {
            match file.read(&mut chunk_buf[got..want]) {
                Ok(0) | Err(_) => break,
                Ok(n) => got += n,
            }
        }
        send_chunk(uart, &chunk_buf[..got]);
        if got < want {
            break;
        }
        remaining -= want;
    }
    Ok(())
}

/// `CMD_SD_WRITE`: receive a file from the host and write it to the FAT
/// boot partition, creating or truncating it.
///
/// Reads a path, then a `u32` LE `total_size` and `u32` LE `chunk_size`,
/// opens the file, then receives the payload as CRC-checked chunks and
/// writes each out. `Err(code)` means the write never started; the
/// caller sends `FAIL` + `code`. After the leading `OK`, chunks are
/// always drained to keep the link in sync even if a write fails, and a
/// final status byte (`OK`, or `FAIL` + [`ERR_WRITE`]) reports the
/// committed result.
fn cmd_sd_write(uart: &mut Uart, timer: &Timer, chunk_buf: &mut [u8]) -> Result<(), u8> {
    let mut path_buf = [0u8; MAX_PATH];
    let path = read_path(uart, &mut path_buf)?;
    let total_size = read_u32_le(uart) as usize;
    let chunk_size = read_u32_le(uart) as usize;
    let (dir_path, file_name) = split_parent(path)?;
    if chunk_size == 0 || chunk_size > chunk_buf.len() {
        return Err(ERR_FS);
    }

    let vm = init_volume_mgr(timer)?;
    let volume = vm.open_volume(VolumeIdx(0)).map_err(err_code)?;
    let mut dir = volume.open_root_dir().map_err(err_code)?;
    for comp in dir_path.split('/').filter(|c| !c.is_empty()) {
        dir.change_dir(comp).map_err(err_code)?;
    }
    let file = dir
        .open_file_in_dir(file_name, Mode::ReadWriteCreateOrTruncate)
        .map_err(err_code)?;

    uart.write_byte(OK);

    let mut offset = 0;
    let mut write_failed = false;
    while offset < total_size {
        let this_len = core::cmp::min(chunk_size, total_size - offset);
        recv_chunk(uart, chunk_buf, this_len);
        // Keep draining chunks even after a write error so the link stays
        // in sync; the failure is reported once, at the end.
        if !write_failed && file.write(&chunk_buf[..this_len]).is_err() {
            write_failed = true;
        }
        // ACK after the write, not before: the `OK` is what frees the host
        // to send the next chunk, so deferring it until we're ready to
        // receive again keeps the SD write from overrunning the RX FIFO
        // (see `recv_chunk`). Sent even on write failure, to keep draining.
        uart.write_byte(OK);
        offset += this_len;
    }

    // Close (not just drop) so a flush error is caught and folded into
    // the final status — the write isn't committed until this returns.
    let close_failed = file.close().is_err();
    if write_failed || close_failed {
        uart.write_byte(FAIL);
        uart.write_byte(ERR_WRITE);
    } else {
        uart.write_byte(OK);
    }
    Ok(())
}

/// `CMD_SD_DELETE`: delete a file from the FAT boot partition.
///
/// Reads a path, walks to its parent directory, and deletes the final
/// component. Replies `OK` on success; `Err(code)` (bad path, not found,
/// SD/FS error) means the caller sends `FAIL` + `code`. Deletes files
/// only — `embedded-sdmmc` rejects deleting a directory as a file, which
/// surfaces here as [`ERR_FS`].
fn cmd_sd_delete(uart: &mut Uart, timer: &Timer) -> Result<(), u8> {
    let mut path_buf = [0u8; MAX_PATH];
    let path = read_path(uart, &mut path_buf)?;
    let (dir_path, file_name) = split_parent(path)?;

    let vm = init_volume_mgr(timer)?;
    let volume = vm.open_volume(VolumeIdx(0)).map_err(err_code)?;
    let mut dir = volume.open_root_dir().map_err(err_code)?;
    for comp in dir_path.split('/').filter(|c| !c.is_empty()) {
        dir.change_dir(comp).map_err(err_code)?;
    }
    dir.delete_file_in_dir(file_name).map_err(err_code)?;

    uart.write_byte(OK);
    Ok(())
}

/// `CMD_SD_MKDIR`: create a directory on the FAT boot partition.
///
/// Reads a path, walks to its parent directory, and creates the final
/// component. Replies `OK` on success; `Err(code)` (bad path, SD/FS
/// error) means the caller sends `FAIL` + `code`. Only the final
/// component is created — the parent directories must already exist, so
/// this is a single `mkdir`, not `mkdir -p`. Creating a directory that
/// already exists surfaces as [`ERR_FS`].
fn cmd_sd_mkdir(uart: &mut Uart, timer: &Timer) -> Result<(), u8> {
    let mut path_buf = [0u8; MAX_PATH];
    let path = read_path(uart, &mut path_buf)?;
    let (dir_path, dir_name) = split_parent(path)?;

    let vm = init_volume_mgr(timer)?;
    let volume = vm.open_volume(VolumeIdx(0)).map_err(err_code)?;
    let mut dir = volume.open_root_dir().map_err(err_code)?;
    for comp in dir_path.split('/').filter(|c| !c.is_empty()) {
        dir.change_dir(comp).map_err(err_code)?;
    }
    dir.make_dir_in_dir(dir_name).map_err(err_code)?;

    uart.write_byte(OK);
    Ok(())
}

/// Brings the SD card up from scratch and wraps it in a
/// [`VolumeManager`] over the FAT filesystem.
///
/// Each SD command calls this fresh: it re-`steal()`s the peripherals it
/// needs and re-runs card identification, trading a little latency for a
/// stateless design with no card handle to thread across the command
/// loop. Stealing again is sound here because the previous command's
/// `Sd` has already been dropped — there's never more than one live at a
/// time on this single core. Maps any bring-up failure to
/// [`ERR_SD_INIT`].
fn init_volume_mgr(timer: &Timer) -> Result<VolumeManager<SdCard<'_>, FixedTime>, u8> {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    // `Sd::steal_emmc` picks the right controller for the active chip --
    // the classic `EMMC` peripheral, or BCM2711's `Emmc2` (not part of
    // `Peripherals` at all, since it isn't in the PAC) -- see rpi-hal's
    // `sd.rs` "BCM2711" doc section.
    let emmc = unsafe { Sd::steal_emmc() };
    let sd = Sd::init(&peripherals.GPIO, emmc, &mut mailbox, timer).map_err(|_| ERR_SD_INIT)?;
    Ok(VolumeManager::new(SdCard::new(sd, timer), FixedTime))
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
/// identical in both.
fn exec(addr: usize) -> ! {
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("dsb", "isb")
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

/// Splits a path into its parent directory and final (file) component.
///
/// Leading/trailing slashes are ignored. `"a/b/c.bin"` → `("a/b",
/// "c.bin")`, `"c.bin"` → `("", "c.bin")`. Returns [`ERR_BAD_PATH`] if
/// there's no final component (e.g. an empty path or bare `/`).
fn split_parent(path: &str) -> Result<(&str, &str), u8> {
    let trimmed = path.trim_matches('/');
    match trimmed.rfind('/') {
        Some(i) => {
            let name = &trimmed[i + 1..];
            if name.is_empty() {
                Err(ERR_BAD_PATH)
            } else {
                Ok((&trimmed[..i], name))
            }
        }
        None if trimmed.is_empty() => Err(ERR_BAD_PATH),
        None => Ok(("", trimmed)),
    }
}

/// Maps an `embedded-sdmmc` error to the coarse code sent to the host.
fn err_code(e: embedded_sdmmc::Error<SdCardError>) -> u8 {
    match e {
        embedded_sdmmc::Error::NotFound => ERR_NOT_FOUND,
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
/// before a slow step (an SD `file.write`, which for a freshly created
/// file allocates a cluster and updates the FAT before programming a
/// block), those incoming bytes are dropped and the framing desyncs
/// permanently. ACKing only once the caller is back at `recv_chunk` turns
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
/// LE chunk size, then the data as [`send_chunk`] chunks. Used by the
/// device→host commands (`CMD_SD_LIST`, and — with the data sourced from
/// a file rather than a slice — `CMD_SD_READ`) after their leading `OK`.
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

/// A fixed timestamp for `embedded-sdmmc`, stamped onto files as they're
/// created or written. A real clock (an RTC or the ARM generic timer) is
/// application policy, not something a loader should impose — and nothing
/// here depends on the timestamp being accurate — so a constant is fine.
struct FixedTime;

impl TimeSource for FixedTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56, // 2026
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// A `core::fmt::Write` sink over a fixed byte buffer used to build a
/// directory listing without an allocator. Once the buffer fills,
/// further writes are dropped and `overflowed` latches true — the caller
/// checks it and rejects the listing rather than sending a truncated one.
struct ListingWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
    overflowed: bool,
}

impl<'a> ListingWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self {
            buf,
            len: 0,
            overflowed: false,
        }
    }
}

impl Write for ListingWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.len >= self.buf.len() {
                self.overflowed = true;
                break;
            }
            self.buf[self.len] = b;
            self.len += 1;
        }
        Ok(())
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
