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
        !matches!(self, Command::Terminal | Command::List { .. })
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
    // to pass to --device, so it cannot require one.
    if let Command::List { all } = &cli.command {
        return list_ports(*all);
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

        Command::Terminal => link.terminal()?,

        // Handled above, before the port was opened.
        Command::List { .. } => unreachable!(),
    }

    Ok(())
}
