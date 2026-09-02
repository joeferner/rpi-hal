//! Host-side driver for rpi-loader: a small on-device agent reached over
//! UART. The loader stays resident and services commands, so this is a
//! subcommand CLI rather than a one-shot uploader — the Pi is
//! power-cycled only to re-flash the loader itself, not between commands.
//!
//! The handshake and terminal always run at [`BASE_BAUD`]; the bulk
//! transfers (`mem-write`, `sd-read`, `sd-write`) optionally negotiate a
//! faster rate (`--baud`) and always drop back before returning, so the
//! next invocation can handshake at the rate the loader is left listening
//! on. The wire protocol itself is documented in [`link`].

mod bundle;
mod link;

use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use anyhow::{anyhow, Context, Result};
use clap::{Parser, Subcommand};
use serialport::{SerialPortInfo, SerialPortType};

use link::{Link, BASE_BAUD, DEFAULT_BAUD};

/// Exit status for a Ctrl-C, matching the shell convention of 128 plus
/// the signal number.
const EXIT_INTERRUPTED: u8 = 130;

/// The 7-bit I2C address the HAT specification assigns the ID EEPROM, as
/// the text `--help` shows and [`parse_i2c_address`] parses.
const HAT_EEPROM_ADDRESS: &str = "0x50";

/// Page size assumed when programming an EEPROM. 32 bytes is the page of
/// a 24C32 — the smallest part the HAT specification allows — and every
/// larger part's page is a multiple of it, so a 32-byte write never
/// crosses a page boundary whatever is fitted.
const DEFAULT_PAGE_SIZE: u32 = 32;

/// Header the HAT specification puts at the start of an ID EEPROM image:
/// `"R-Pi"`, a format version and a reserved byte, the atom count, then
/// the image length. Only the signature and that length are read here —
/// enough for `eeprom-read` to work out how much to read.
const HAT_HEADER_LEN: u32 = 12;
/// Magic the header starts with.
const HAT_SIGNATURE: &[u8; 4] = b"R-Pi";
/// Where the image length sits within the header.
const HAT_EEPLEN_AT: usize = 8;
/// Ceiling on an image length taken from the header, matching what the
/// device's two-byte addressing can reach — a corrupt header should be an
/// error, not a request to read 4 GiB one chunk at a time.
const HAT_MAX_IMAGE: u32 = 0x1_0000;

/// Upload firmware to a Raspberry Pi over serial, and read or write its
/// SD card, without touching the card itself.
#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    // `global` so it is accepted on either side of the subcommand:
    // `rpi-loader --device /dev/ttyUSB0 boot img` and
    // `rpi-loader boot img --device /dev/ttyUSB0` are the same command.
    // clap forbids `required` on a global argument, so it is an option
    // here and `run` rejects a missing one itself. Doc comments on these
    // fields are what `--help` prints, so this note stays a plain
    // comment.
    /// Serial device the loader is attached to (e.g. /dev/ttyUSB0).
    #[arg(short, long, global = true, value_name = "DEVICE")]
    device: Option<String>,

    /// What to ask the loader to do.
    #[command(subcommand)]
    command: Command,
}

/// The subcommands, one per loader operation.
#[derive(Subcommand)]
enum Command {
    /// Upload an image to memory, jump to it, then act as a terminal.
    Boot {
        /// Kernel image to upload.
        image: PathBuf,
        /// Load address: 0x8000 for a 32-bit kernel7.img, 0x80000 for a
        /// 64-bit kernel8.img.
        #[arg(long, value_parser = parse_u32)]
        load_addr: u32,
        /// Baud to negotiate for the transfer; the link always returns to
        /// 115200 afterward.
        #[arg(long, value_parser = parse_u32, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },

    /// Write a file to a memory address (no jump).
    MemWrite {
        /// Destination address (e.g. 0x8000).
        #[arg(value_parser = parse_u32)]
        addr: u32,
        /// File to write.
        file: PathBuf,
        /// Baud to negotiate for the transfer; the link always returns to
        /// 115200 afterward.
        #[arg(long, value_parser = parse_u32, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },

    /// Jump to an address already loaded in memory.
    Exec {
        /// Address to jump to (e.g. 0x8000).
        #[arg(value_parser = parse_u32)]
        addr: u32,
        /// Stay attached as a passthrough terminal after jumping.
        #[arg(long)]
        terminal: bool,
    },

    /// List a directory on the SD card's FAT boot partition.
    SdList {
        /// Directory path (default /).
        #[arg(default_value = "/")]
        path: String,
    },

    /// Copy a file off the SD card.
    SdRead {
        /// Path on the SD card (e.g. /config.txt).
        remote: String,
        /// Local file to write.
        local: PathBuf,
        /// Baud to negotiate for the transfer; the link always returns to
        /// 115200 afterward.
        #[arg(long, value_parser = parse_u32, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },

    /// Copy a local file onto the SD card, creating or truncating it.
    SdWrite {
        /// Local file to read.
        local: PathBuf,
        /// Path on the SD card (e.g. /TEST.BIN).
        remote: String,
        /// Baud to negotiate for the transfer; the link always returns to
        /// 115200 afterward.
        #[arg(long, value_parser = parse_u32, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },

    /// Delete a file from the SD card.
    SdDelete {
        /// Path on the SD card (e.g. /TEST.BIN).
        remote: String,
    },

    /// Create a directory on the SD card (a single level).
    SdMkdir {
        /// Directory path on the SD card (e.g. /LOGS).
        remote: String,
    },

    /// Copy an EEPROM's contents off the HAT ID bus (GPIO0/1) into a file.
    EepromRead {
        /// Local file to write.
        local: PathBuf,
        /// How many bytes to read. Default: the image length from the HAT
        /// EEPROM header, which requires the EEPROM to hold one.
        #[arg(long, value_parser = parse_u32)]
        length: Option<u32>,
        /// 7-bit I2C address. 0x50 is what the HAT specification assigns
        /// the ID EEPROM.
        // The default is spelled as the string `--help` should show: with
        // `default_value_t` clap prints the `u8`, and "80" is a poor way
        // to write an I2C address every datasheet gives as 0x50.
        #[arg(long, value_parser = parse_i2c_address, default_value = HAT_EEPROM_ADDRESS)]
        address: u8,
        /// Byte offset to start at.
        #[arg(long, value_parser = parse_u32, default_value_t = 0)]
        offset: u32,
        /// Baud to negotiate for the transfer; the link always returns to
        /// 115200 afterward.
        #[arg(long, value_parser = parse_u32, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },

    /// Program a local image into an EEPROM on the HAT ID bus (GPIO0/1).
    EepromWrite {
        /// Local image to program — for a HAT ID EEPROM, the `.eep` file
        /// Raspberry Pi's `eepmake` produces.
        local: PathBuf,
        /// 7-bit I2C address. 0x50 is what the HAT specification assigns
        /// the ID EEPROM.
        // The default is spelled as the string `--help` should show: with
        // `default_value_t` clap prints the `u8`, and "80" is a poor way
        // to write an I2C address every datasheet gives as 0x50.
        #[arg(long, value_parser = parse_i2c_address, default_value = HAT_EEPROM_ADDRESS)]
        address: u8,
        /// Byte offset to start at.
        #[arg(long, value_parser = parse_u32, default_value_t = 0)]
        offset: u32,
        /// The part's page size in bytes. The default suits every part
        /// from the 24C32 (the HAT specification's floor) up; a 24C256's
        /// own page is 64 bytes, which programs in half the time. Too
        /// large corrupts data rather than failing, since a page write
        /// that overruns wraps to the start of the same page.
        #[arg(long, value_parser = parse_u32, default_value_t = DEFAULT_PAGE_SIZE)]
        page_size: u32,
        /// Baud to negotiate for the transfer; the link always returns to
        /// 115200 afterward.
        #[arg(long, value_parser = parse_u32, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },

    /// Pack an over-the-air bundle from a manifest, and optionally upload
    /// it to a running board.
    ///
    /// The only subcommand that touches no serial port: a bundle reaches a
    /// board over the network, and this is here because the format has to
    /// have one implementation, shared with the firmware that installs it.
    Bundle {
        /// The manifest describing what the bundle holds.
        #[arg(default_value = "bundle.toml")]
        manifest: PathBuf,
        /// Where to write it. Defaults to `target/<name>.bundle` beside
        /// the manifest.
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// POST it to a running board once built, e.g.
        /// http://10.0.0.5/api/v1/ota.
        #[arg(long, value_name = "URL")]
        upload: Option<String>,
    },

    /// Passthrough serial terminal only, with no handshake.
    Terminal,

    /// List the serial ports on this machine, to find the one to pass to
    /// --device.
    List {
        /// Include non-USB ports too — the legacy /dev/ttyS* range, PCI
        /// and Bluetooth serial. A USB-to-serial cable is what the loader
        /// is reached through, so these are filtered out by default;
        /// a typical Linux machine reports thirty-odd of them.
        #[arg(long)]
        all: bool,
    },
}

impl Command {
    /// Whether this command must greet the loader before running.
    ///
    /// `terminal` is the exception among the ones that open the port: it
    /// exists to watch an already-running kernel, which doesn't speak the
    /// loader protocol, so a HELLO would never be answered.
    fn needs_handshake(&self) -> bool {
        !matches!(
            self,
            Command::Terminal | Command::List { .. } | Command::Bundle { .. }
        )
    }
}

/// Parses an address or baud in any of the bases a user is likely to
/// type, mirroring what the shell and the datasheets use: `0x8000`,
/// `0b1010`, `0o755`, `1500000`, or `1_500_000`.
fn parse_u32(s: &str) -> Result<u32, String> {
    let text = s.trim().replace('_', "");
    let (digits, radix) = match text.get(..2).map(str::to_ascii_lowercase).as_deref() {
        Some("0x") => (&text[2..], 16),
        Some("0b") => (&text[2..], 2),
        Some("0o") => (&text[2..], 8),
        _ => (&text[..], 10),
    };
    u32::from_str_radix(digits, radix).map_err(|e| format!("{s:?} is not a number: {e}"))
}

/// Parses a 7-bit I2C address in any of the bases [`parse_u32`] takes,
/// rejecting anything the bus cannot carry. 0x00-0x07 and 0x78-0x7f are
/// reserved by the I2C specification, but a part answering there is the
/// user's business, not this tool's — only the 7-bit range is enforced.
fn parse_i2c_address(s: &str) -> Result<u8, String> {
    let value = parse_u32(s)?;
    u8::try_from(value)
        .ok()
        .filter(|&address| address <= 0x7f)
        .ok_or_else(|| format!("{s:?} is not a 7-bit I2C address (0x00-0x7f)"))
}

/// Reads a local file, naming it if that fails.
fn read_file(path: &Path) -> Result<Vec<u8>> {
    fs::read(path).with_context(|| format!("reading {}", path.display()))
}

/// Describes a port's type in one line: for USB, whatever the device
/// reports about itself, which is what distinguishes one cable from
/// another when several are plugged in.
fn describe(port_type: &SerialPortType) -> String {
    match port_type {
        SerialPortType::UsbPort(info) => {
            let mut parts = Vec::new();
            let name = [info.manufacturer.as_deref(), info.product.as_deref()]
                .into_iter()
                .flatten()
                .collect::<Vec<_>>()
                .join(" ");
            if !name.is_empty() {
                parts.push(name);
            }
            parts.push(format!("{:04x}:{:04x}", info.vid, info.pid));
            if let Some(serial) = &info.serial_number {
                parts.push(format!("serial {serial}"));
            }
            parts.join("  ")
        }
        SerialPortType::PciPort => "PCI serial".into(),
        SerialPortType::BluetoothPort => "Bluetooth serial".into(),
        _ => "unknown".into(),
    }
}

/// Lists the serial ports this machine can see.
///
/// Enumeration needs no privileges and opens nothing, so this works even
/// where the port itself would be refused for lack of group membership.
fn list_ports(all: bool) -> Result<()> {
    let mut ports = serialport::available_ports().context("enumerating serial ports")?;
    if !all {
        ports.retain(|port| matches!(port.port_type, SerialPortType::UsbPort(_)));
    }
    // USB first, then by name: with --all the handful of interesting
    // ports would otherwise be buried in the legacy range.
    ports.sort_by(|a, b| {
        let rank = |port: &SerialPortInfo| !matches!(port.port_type, SerialPortType::UsbPort(_));
        (rank(a), a.port_name.clone()).cmp(&(rank(b), b.port_name.clone()))
    });

    if ports.is_empty() {
        eprintln!(
            "{}",
            if all {
                "No serial ports found."
            } else {
                "No USB serial ports found; pass --all to list every port."
            }
        );
        return Ok(());
    }

    let width = ports
        .iter()
        .map(|port| port.port_name.len())
        .max()
        .unwrap_or(0);
    for port in &ports {
        println!(
            "{:width$}  {}",
            port.port_name,
            describe(&port.port_type),
            width = width
        );
    }
    Ok(())
}

/// Works out how much of an EEPROM to read when `--length` was not given,
/// by reading the HAT header at `offset` and taking the image length out
/// of it.
///
/// The alternative would be reading the whole address space, which for a
/// 32 KiB part means seconds of I2C traffic to recover a few hundred bytes
/// of atoms and 31 KiB of `0xff`. An EEPROM without a valid header is
/// asked for by length instead — the error says so.
fn hat_image_length(link: &mut Link, address: u8, offset: u32) -> Result<u32> {
    let header = link.eeprom_read(address, offset, HAT_HEADER_LEN)?;
    if !header.starts_with(HAT_SIGNATURE) {
        return Err(anyhow!(
            "0x{address:02x} does not hold a HAT image (no \"R-Pi\" signature at offset \
             {offset}); pass --length to read it anyway"
        ));
    }
    let eeplen = u32::from_le_bytes(
        header[HAT_EEPLEN_AT..HAT_EEPLEN_AT + 4]
            .try_into()
            .expect("the header is 12 bytes, so this slice is 4"),
    );
    if !(HAT_HEADER_LEN..=HAT_MAX_IMAGE).contains(&eeplen) {
        return Err(anyhow!(
            "the HAT header claims an image length of {eeplen} bytes, which is not \
             plausible; pass --length to read it anyway"
        ));
    }
    Ok(eeplen)
}

fn main() -> ExitCode {
    let cli = Cli::parse();

    // Ctrl-C sets a flag rather than killing the process, so the terminal
    // can leave cleanly and a transfer can unwind instead of stranding
    // the device mid-chunk. Every loop that could block forever polls it.
    let interrupted = Arc::new(AtomicBool::new(false));
    let flag = Arc::clone(&interrupted);
    if let Err(e) = ctrlc::set_handler(move || flag.store(true, Ordering::SeqCst)) {
        eprintln!("Warning: could not install the Ctrl-C handler: {e}");
    }

    match run(cli, Arc::clone(&interrupted)) {
        Ok(()) if interrupted.load(Ordering::SeqCst) => ExitCode::from(EXIT_INTERRUPTED),
        Ok(()) => ExitCode::SUCCESS,
        // The error here is whatever loop noticed the flag and unwound;
        // saying "interrupted" is more use than reporting it.
        Err(_) if interrupted.load(Ordering::SeqCst) => {
            eprintln!("\nInterrupted.");
            ExitCode::from(EXIT_INTERRUPTED)
        }
        Err(e) => {
            eprintln!("Error: {e:#}");
            ExitCode::FAILURE
        }
    }
}

/// Opens the port, greets the loader unless the command opts out, and
/// runs the command.
fn run(cli: Cli, interrupted: Arc<AtomicBool>) -> Result<()> {
    // Before anything opens a port: listing is how a user finds out what
    // to pass to --device, so it cannot require one, and packing a bundle
    // never involves the serial link at all.
    if let Command::List { all } = &cli.command {
        return list_ports(*all);
    }
    if let Command::Bundle {
        manifest,
        output,
        upload,
    } = &cli.command
    {
        return bundle::run(manifest, output.clone(), upload.as_deref());
    }

    let device = cli.device.as_deref().ok_or_else(|| {
        anyhow!("no serial device given; pass --device (e.g. --device /dev/ttyUSB0)")
    })?;
    let mut link = Link::open(device, interrupted)?;
    if cli.command.needs_handshake() {
        link.handshake()?;
    }

    match cli.command {
        Command::Boot {
            image,
            load_addr,
            baud,
        } => {
            let data = read_file(&image)?;
            link.negotiate_baud(baud)?;
            link.mem_write(load_addr, &data)?;
            // Drop back to base baud before jumping so a booted kernel's
            // output lands at the rate the terminal below listens at.
            link.negotiate_baud(BASE_BAUD)?;
            link.exec(load_addr)?;
            eprintln!("Jumped to {load_addr:#x}.");
            link.terminal()?;
        }

        Command::MemWrite { addr, file, baud } => {
            let data = read_file(&file)?;
            link.negotiate_baud(baud)?;
            link.mem_write(addr, &data)?;
            link.negotiate_baud(BASE_BAUD)?;
            eprintln!("Wrote {} bytes to {addr:#x}.", data.len());
        }

        Command::Exec { addr, terminal } => {
            link.exec(addr)?;
            eprintln!("Jumped to {addr:#x}.");
            if terminal {
                link.terminal()?;
            }
        }

        Command::SdList { path } => {
            for entry in link.sd_list(&path)? {
                let marker = if entry.is_dir { "/" } else { "" };
                println!("{:>12}  {}{marker}", entry.size, entry.name);
            }
        }

        Command::SdRead {
            remote,
            local,
            baud,
        } => {
            link.negotiate_baud(baud)?;
            let data = link.sd_read(&remote)?;
            link.negotiate_baud(BASE_BAUD)?;
            fs::write(&local, &data).with_context(|| format!("writing {}", local.display()))?;
            eprintln!(
                "Read {} bytes from {remote} -> {}",
                data.len(),
                local.display()
            );
        }

        Command::SdWrite {
            local,
            remote,
            baud,
        } => {
            let data = read_file(&local)?;
            link.negotiate_baud(baud)?;
            link.sd_write(&remote, &data)?;
            link.negotiate_baud(BASE_BAUD)?;
        }

        Command::SdDelete { remote } => {
            link.sd_delete(&remote)?;
            eprintln!("Deleted {remote}");
        }

        Command::SdMkdir { remote } => {
            link.sd_mkdir(&remote)?;
            eprintln!("Created directory {remote}");
        }

        Command::EepromRead {
            local,
            length,
            address,
            offset,
            baud,
        } => {
            link.negotiate_baud(baud)?;
            let length = match length {
                Some(length) => length,
                None => hat_image_length(&mut link, address, offset)?,
            };
            let data = link.eeprom_read(address, offset, length)?;
            link.negotiate_baud(BASE_BAUD)?;
            fs::write(&local, &data).with_context(|| format!("writing {}", local.display()))?;
            eprintln!(
                "Read {} bytes from 0x{address:02x} -> {}",
                data.len(),
                local.display()
            );
        }

        Command::EepromWrite {
            local,
            address,
            offset,
            page_size,
            baud,
        } => {
            let data = read_file(&local)?;
            if data.is_empty() {
                return Err(anyhow!("{} is empty", local.display()));
            }
            // A warning rather than a refusal: this command programs an
            // EEPROM, and only the usual one on this bus holds a HAT image.
            if offset == 0 && !data.starts_with(HAT_SIGNATURE) {
                eprintln!(
                    "Warning: {} does not start with the HAT signature \"R-Pi\"; \
                     writing it anyway.",
                    local.display()
                );
            }
            link.negotiate_baud(baud)?;
            link.eeprom_write(address, offset, page_size, &data)?;
            link.negotiate_baud(BASE_BAUD)?;
            eprintln!(
                "Programmed and verified {} bytes at offset {offset} of 0x{address:02x}.",
                data.len()
            );
        }

        Command::Terminal => link.terminal()?,

        // Both returned above, before a port was ever opened.
        Command::Bundle { .. } => unreachable!(),

        // Handled above, before the port was opened.
        Command::List { .. } => unreachable!(),
    }

    Ok(())
}
