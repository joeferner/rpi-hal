//! Building an over-the-air bundle from a manifest, and posting it to a
//! board.
//!
//! The container itself lives in `rpi-loader-ota`, which the device links
//! too — so what is here is only the half that turns a description of a
//! project's card into the entries that crate encodes.
//!
//! # Why a manifest rather than arguments
//!
//! What goes into a bundle is a property of the project, not of the
//! invocation. Arguments could just about carry a kernel and one directory,
//! which is what the shell scripts this replaces did; they cannot carry a
//! settings file, a firmware blob and a certificate with a destination
//! each. The moment they try, a project's card layout lives in its
//! `Makefile` in a form nothing can check.

use std::collections::BTreeMap;
use std::fs;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use rpi_loader_ota::{encode, Bundle, Entry, Format, Role};
use serde::Deserialize;

/// Entries a bundle may hold unless the manifest says otherwise.
///
/// The receiving firmware has its own bound and rejects a bundle that
/// exceeds it, so this is the packer noticing first rather than the only
/// check. A project whose firmware allows more says so in its manifest.
const DEFAULT_MAX_ENTRIES: usize = 32;

/// How long to wait on an upload before giving up.
///
/// Generous on purpose: the request does not return until the board has
/// written every entry to its card and read them all back, so the reply is
/// paced by the slowest thing in the system rather than by the network.
const UPLOAD_TIMEOUT: Duration = Duration::from_secs(300);

/// A project's `bundle.toml`: what its card holds.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    /// Four ASCII bytes identifying the application, matching what its
    /// firmware was built with.
    magic: String,
    /// Names the default output file, `target/<name>.bundle`.
    name: String,
    /// Entry ceiling, if this project's firmware differs from the default.
    max_entries: Option<usize>,
    /// The boot image. Absent for an update that replaces no kernel.
    kernel: Option<KernelSpec>,
    /// Everything else.
    #[serde(default)]
    files: Vec<FileSpec>,
}

/// The `[kernel]` table.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct KernelSpec {
    /// The built image, e.g. `target/kernel7.img`.
    source: PathBuf,
    /// What to call it in the bundle. Defaults to the source's file name.
    ///
    /// A label rather than an instruction: an installer running two kernel
    /// slots picks the destination itself, because it is the half that
    /// knows which slot is live. It is carried because every entry has a
    /// path, and because a log naming the file is more use than one naming
    /// a role.
    dest: Option<String>,
}

/// One `[[files]]` entry: a file, or a directory of them.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct FileSpec {
    /// A file to pack, or a directory whose contents to pack.
    source: PathBuf,
    /// Where it lands on the card. For a directory, the directory it lands
    /// in. Defaults to the source's file name.
    dest: Option<String>,
    /// How the device should treat it. Defaults to `file`.
    #[serde(default)]
    role: FileRole,
}

/// The roles a `[[files]]` entry may claim.
#[derive(Deserialize, Clone, Copy, Default, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
enum FileRole {
    /// Anything the application owns.
    #[default]
    File,
    /// A Raspberry Pi firmware file.
    Firmware,
    /// `config.txt`.
    Config,
    /// Rejected — the boot image has its own table, and accepting it here
    /// would be a second way to say the same thing and a way to say it
    /// twice.
    Kernel,
}

/// A file read off disk, owning what [`Entry`] borrows.
struct Loaded {
    role: Role,
    path: String,
    data: Vec<u8>,
}

/// Builds the bundle `manifest` describes, writes it, and optionally
/// unpacks it onto a card or posts it to a running board.
pub fn run(
    manifest: &Path,
    output: Option<PathBuf>,
    sdcard: Option<&Path>,
    upload: Option<&str>,
) -> Result<()> {
    let text = fs::read_to_string(manifest)
        .with_context(|| format!("reading the manifest {}", manifest.display()))?;
    let parsed: Manifest = toml::from_str(&text)
        .with_context(|| format!("parsing the manifest {}", manifest.display()))?;

    // Sources are relative to the manifest, not to wherever this was run
    // from, so `rpi-loader bundle ../water/bundle.toml` means the same
    // thing as running it in that directory.
    let root = manifest.parent().unwrap_or(Path::new("."));

    let format = Format {
        magic: parse_magic(&parsed.magic)?,
        max_entries: parsed.max_entries.unwrap_or(DEFAULT_MAX_ENTRIES),
    };

    let loaded = load(&parsed, root)?;
    let entries: Vec<Entry<'_>> = loaded
        .iter()
        .map(|file| Entry {
            role: file.role,
            path: &file.path,
            data: &file.data,
        })
        .collect();

    let bytes = encode(&format, &entries)?;

    let out = output.unwrap_or_else(|| root.join("target").join(format!("{}.bundle", parsed.name)));
    if let Some(parent) = out.parent() {
        fs::create_dir_all(parent).with_context(|| format!("creating {}", parent.display()))?;
    }
    fs::write(&out, &bytes).with_context(|| format!("writing {}", out.display()))?;

    report(&out, &bytes, &loaded);

    if let Some(directory) = sdcard {
        unpack(&format, &bytes, directory)?;
    }
    match upload {
        Some(url) => post(url, &bytes),
        None => Ok(()),
    }
}

/// Writes the bundle's contents onto a mounted card.
///
/// For the update a board cannot be sent: a build that changes the bundle
/// format it reads, or one that broke networking, or a first install. The
/// card goes in a reader and comes out holding what an over-the-air update
/// would have put there.
///
/// **This writes what the bundle carries, which is deliberately less than a
/// card needs to boot.** A manifest names what an *update* replaces; the
/// Raspberry Pi firmware, and any settings file a project excludes on
/// purpose so that updates do not reset it, are not in it and are not
/// written here.
///
/// # Why it decodes what it just encoded
///
/// The entries are in hand already, so writing them directly would be
/// shorter. Going back through [`Bundle::parse`] means the card is written
/// from the same bytes a device would install, checked by the same code —
/// so "I flashed it by hand" and "I sent it over the network" cannot come
/// to different answers. It also means the paths have been validated
/// against escaping their directory before any of them is joined to a path
/// on this machine.
fn unpack(format: &Format, bytes: &[u8], directory: &Path) -> Result<()> {
    // Refused rather than created. A mount point exists; a typo does not,
    // and silently making one produces a card that looks written and a
    // directory of files nobody will find again.
    if !directory.is_dir() {
        bail!(
            "{} is not a directory — is the card mounted?",
            directory.display()
        );
    }

    let bundle = Bundle::parse(format, bytes)?;
    println!(
        "\nwriting {} entries to {}",
        bundle.count(),
        directory.display()
    );

    for entry in bundle.iter() {
        // Component by component rather than joining the whole path: a
        // bundle's separator is always `/`, and this machine's may not be.
        let mut destination = directory.to_path_buf();
        for component in entry.path.split('/') {
            destination.push(component);
        }
        if let Some(parent) = destination.parent() {
            fs::create_dir_all(parent).with_context(|| format!("creating {}", parent.display()))?;
        }

        // Written and flushed to the device, not merely to the page cache.
        // Without the sync this prints "done" while the card still holds
        // none of it, and the next thing anyone does is pull the card out.
        let mut file = fs::File::create(&destination)
            .with_context(|| format!("creating {}", destination.display()))?;
        file.write_all(entry.data)
            .with_context(|| format!("writing {}", destination.display()))?;
        file.sync_all()
            .with_context(|| format!("flushing {}", destination.display()))?;

        println!("  {:>9}  {}", entry.data.len(), entry.path);
    }

    Ok(())
}

/// Reads every source the manifest names.
fn load(manifest: &Manifest, root: &Path) -> Result<Vec<Loaded>> {
    let mut loaded = Vec::new();

    if let Some(kernel) = &manifest.kernel {
        let source = root.join(&kernel.source);
        let path = match &kernel.dest {
            Some(dest) => dest.clone(),
            None => file_name(&source)?,
        };
        loaded.push(Loaded {
            role: Role::Kernel,
            path,
            data: read(&source)?,
        });
    }

    for spec in &manifest.files {
        let role = match spec.role {
            FileRole::File => Role::File,
            FileRole::Firmware => Role::Firmware,
            FileRole::Config => Role::Config,
            FileRole::Kernel => bail!(
                "{}: role = \"kernel\" is not allowed in [[files]]; \
                 name the boot image in the [kernel] table instead",
                spec.source.display()
            ),
        };
        let source = root.join(&spec.source);
        let dest = match &spec.dest {
            Some(dest) => dest.clone(),
            None => file_name(&source)?,
        };

        if source.is_dir() {
            for (relative, data) in walk(&source)? {
                loaded.push(Loaded {
                    role,
                    path: format!("{dest}/{relative}"),
                    data,
                });
            }
        } else {
            loaded.push(Loaded {
                role,
                path: dest,
                data: read(&source)?,
            });
        }
    }

    Ok(loaded)
}

/// Every regular file under `directory`, keyed by its path relative to it.
///
/// A `BTreeMap` because the order entries are packed in has to depend only
/// on their names: two runs over the same tree should produce the same
/// bytes, and a directory listing is in whatever order the filesystem
/// happens to return.
///
/// Recursive, unlike the scripts this replaces, because a bundle can now
/// carry nested destinations and a `www/` with a subdirectory in it is an
/// ordinary thing to want. Anything beginning with a dot is skipped at
/// every level, which is what keeps a `.git` out of a bundle.
fn walk(directory: &Path) -> Result<BTreeMap<String, Vec<u8>>> {
    let mut found = BTreeMap::new();
    collect(directory, "", &mut found)?;
    Ok(found)
}

/// Adds everything under `directory` to `into`, prefixing with `prefix`.
fn collect(directory: &Path, prefix: &str, into: &mut BTreeMap<String, Vec<u8>>) -> Result<()> {
    let listing =
        fs::read_dir(directory).with_context(|| format!("reading {}", directory.display()))?;
    for entry in listing {
        let entry = entry.with_context(|| format!("reading {}", directory.display()))?;
        let name = entry.file_name().to_string_lossy().into_owned();
        if name.starts_with('.') {
            continue;
        }
        // Always `/`, never the platform's separator: what goes into a
        // bundle is a path on a FAT volume, not a path on this machine.
        let relative = if prefix.is_empty() {
            name
        } else {
            format!("{prefix}/{name}")
        };
        let path = entry.path();
        if path.is_dir() {
            collect(&path, &relative, into)?;
        } else {
            into.insert(relative, read(&path)?);
        }
    }
    Ok(())
}

/// Reads a file, naming it if that fails.
fn read(path: &Path) -> Result<Vec<u8>> {
    fs::read(path).with_context(|| format!("reading {}", path.display()))
}

/// The file name of `path`, for a manifest entry that gave no `dest`.
fn file_name(path: &Path) -> Result<String> {
    path.file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .with_context(|| {
            format!(
                "{} has no file name to use as a destination",
                path.display()
            )
        })
}

/// Parses the manifest's magic into the four bytes a bundle header holds.
fn parse_magic(magic: &str) -> Result<[u8; 4]> {
    let bytes = magic.as_bytes();
    if bytes.len() != 4 || !magic.is_ascii() {
        bail!("magic must be exactly four ASCII characters, not {magic:?}");
    }
    Ok([bytes[0], bytes[1], bytes[2], bytes[3]])
}

/// Prints what was packed.
fn report(out: &Path, bytes: &[u8], loaded: &[Loaded]) {
    println!(
        "{} ({} bytes, {} entries)",
        out.display(),
        bytes.len(),
        loaded.len()
    );
    let width = loaded.iter().map(|file| file.path.len()).max().unwrap_or(0);
    for file in loaded {
        let role = match file.role {
            Role::File => "",
            Role::Kernel => "  kernel",
            Role::Firmware => "  firmware",
            Role::Config => "  config",
        };
        println!(
            "  {:<width$}  {:>9}{role}",
            file.path,
            file.data.len(),
            width = width
        );
    }
}

/// Posts `bytes` to a running board and prints what it says.
///
/// Plain HTTP, with no TLS anywhere in the dependency tree. The endpoint is
/// a board on a local network that speaks `http://`, and pulling a TLS
/// stack into a serial tool to reach it would cost more than it protects.
fn post(url: &str, bytes: &[u8]) -> Result<()> {
    println!("\nuploading {} bytes to {url}", bytes.len());

    let agent = ureq::Agent::config_builder()
        .timeout_global(Some(UPLOAD_TIMEOUT))
        // The device answers a rejected bundle with a body saying why, and
        // that body is the most useful thing it can send. Left as an error
        // by default, ureq would discard it.
        .http_status_as_error(false)
        .build()
        .new_agent();

    let started = Instant::now();
    let mut response = agent
        .post(url)
        .content_type("application/octet-stream")
        .send(bytes)
        .with_context(|| format!("posting to {url}"))?;
    let status = response.status();

    let mut body = String::new();
    response
        .body_mut()
        .as_reader()
        .read_to_string(&mut body)
        .with_context(|| format!("reading the reply from {url}"))?;

    println!(
        "{status}, {:.1}s round trip",
        started.elapsed().as_secs_f64()
    );
    match serde_json::from_str::<serde_json::Value>(&body) {
        Ok(json) => println!("{}", serde_json::to_string_pretty(&json)?),
        Err(_) => println!("{}", body.trim_end()),
    }

    if !status.is_success() {
        bail!("the board rejected the bundle ({status})");
    }
    Ok(())
}
