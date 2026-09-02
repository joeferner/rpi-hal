//! `rpi-loader bundle` against real files on disk.
//!
//! These run the real binary over a real directory tree, because almost
//! everything the subcommand does is about the filesystem: which files a
//! directory source picks up, what they are called once they are in the
//! bundle, and in what order. None of that is visible from a unit test of
//! the manifest types.
//!
//! What the bundle *contains* is then checked by parsing it with
//! `rpi-loader-ota` — the same code the firmware runs — so a test failing
//! here means the packer and the installer disagree, which is the one
//! failure this whole crate exists to make impossible.

use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use rpi_loader_ota::{Bundle, Entry, Format, Role};

/// A scratch directory that cleans up after itself.
struct Project {
    root: PathBuf,
}

impl Project {
    /// Creates one named after the test using it, so a leftover directory
    /// says which test left it.
    fn new(name: &str) -> Project {
        let root = std::env::temp_dir().join(format!("rpi-loader-bundle-{name}"));
        let _ = fs::remove_dir_all(&root);
        fs::create_dir_all(&root).expect("creating the scratch directory");
        Project { root }
    }

    /// Writes a file, creating the directories above it.
    fn write(&self, relative: &str, contents: &[u8]) -> &Project {
        let path = self.root.join(relative);
        fs::create_dir_all(path.parent().expect("a parent")).expect("creating a directory");
        fs::write(path, contents).expect("writing a file");
        self
    }

    /// Runs `rpi-loader bundle` in this project.
    fn pack(&self, arguments: &[&str]) -> Output {
        Command::new(env!("CARGO_BIN_EXE_rpi-loader"))
            .arg("bundle")
            .arg(self.root.join("bundle.toml"))
            .args(arguments)
            .output()
            .expect("running the packer")
    }

    /// The bundle the default output path names.
    fn output(&self, name: &str) -> PathBuf {
        self.root.join("target").join(format!("{name}.bundle"))
    }
}

impl Drop for Project {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.root);
    }
}

/// Parses a packed bundle the way a device would, and hands back what a
/// device would see.
fn entries(path: &Path, magic: &[u8; 4]) -> Vec<(Role, String, Vec<u8>)> {
    let bytes = fs::read(path).expect("reading the packed bundle");
    let format = Format {
        magic: *magic,
        max_entries: 32,
    };
    let bundle = Bundle::parse(&format, &bytes).expect("the packed bundle should parse");
    bundle
        .iter()
        .map(|Entry { role, path, data }| (role, path.to_owned(), data.to_vec()))
        .collect()
}

/// The message a failed run printed, for an assertion to name.
fn stderr(output: &Output) -> String {
    String::from_utf8_lossy(&output.stderr).into_owned()
}

#[test]
fn packs_a_whole_project() {
    let project = Project::new("whole");
    project
        .write(
            "bundle.toml",
            br#"
                magic = "WATR"
                name  = "water"

                [kernel]
                source = "target/kernel7.img"
                dest   = "KERNEL7.IMG"

                [[files]]
                source = "www"
                dest   = "WWW"

                [[files]]
                source = "vendor/start.elf"
                dest   = "START.ELF"
                role   = "firmware"

                [[files]]
                source = "vendor/fixup.dat"
                dest   = "FIXUP.DAT"
                role   = "firmware"

                [[files]]
                source = "config.txt"
                dest   = "CONFIG.TXT"
                role   = "config"
            "#,
        )
        .write("target/kernel7.img", b"kernel")
        .write("www/index.htm", b"<html>")
        .write("www/css/site.css", b"body{}")
        .write("www/.hidden", b"not this one")
        .write("vendor/start.elf", b"start")
        .write("vendor/fixup.dat", b"fixup")
        .write("config.txt", b"arm_64bit=0\n");

    let output = project.pack(&[]);
    assert!(output.status.success(), "{}", stderr(&output));

    assert_eq!(
        entries(&project.output("water"), b"WATR"),
        vec![
            (Role::Kernel, "KERNEL7.IMG".into(), b"kernel".to_vec()),
            // Nested, because a bundle can carry a path now — and sorted,
            // so the same tree always packs to the same bytes.
            (Role::File, "WWW/css/site.css".into(), b"body{}".to_vec()),
            (Role::File, "WWW/index.htm".into(), b"<html>".to_vec()),
            (Role::Firmware, "START.ELF".into(), b"start".to_vec()),
            (Role::Firmware, "FIXUP.DAT".into(), b"fixup".to_vec()),
            (Role::Config, "CONFIG.TXT".into(), b"arm_64bit=0\n".to_vec()),
        ],
        "a dotfile was packed, or the order was not what the tree says"
    );
}

#[test]
fn a_bundle_needs_no_kernel() {
    let project = Project::new("kernelless");
    project
        .write(
            "bundle.toml",
            br#"
                magic = "BSMT"
                name  = "basement"

                [[files]]
                source = "settings.cfg"
                dest   = "SETTINGS.CFG"
            "#,
        )
        .write("settings.cfg", b"zone=1");

    let output = project.pack(&[]);
    assert!(output.status.success(), "{}", stderr(&output));
    assert_eq!(
        entries(&project.output("basement"), b"BSMT"),
        vec![(Role::File, "SETTINGS.CFG".into(), b"zone=1".to_vec())]
    );
}

#[test]
fn a_destination_defaults_to_the_source_file_name() {
    let project = Project::new("default-dest");
    project
        .write(
            "bundle.toml",
            br#"
                magic = "WATR"
                name  = "water"

                [[files]]
                source = "vendor/CA.PEM"
            "#,
        )
        .write("vendor/CA.PEM", b"-----BEGIN");

    let output = project.pack(&[]);
    assert!(output.status.success(), "{}", stderr(&output));
    assert_eq!(
        entries(&project.output("water"), b"WATR")[0].1,
        "CA.PEM".to_owned()
    );
}

#[test]
fn output_can_be_named() {
    let project = Project::new("named-output");
    project
        .write(
            "bundle.toml",
            br#"
                magic = "WATR"
                name  = "water"

                [[files]]
                source = "a.txt"
                dest   = "A.TXT"
            "#,
        )
        .write("a.txt", b"a");

    let elsewhere = project.root.join("build/custom.bin");
    let output = project.pack(&["--output", elsewhere.to_str().expect("a path")]);
    assert!(output.status.success(), "{}", stderr(&output));
    // The directory did not exist: a packer that made the caller create it
    // first would be a step in every Makefile that uses one.
    assert!(elsewhere.is_file());
}

#[test]
fn the_magic_must_be_four_characters() {
    let project = Project::new("short-magic");
    project
        .write(
            "bundle.toml",
            br#"
                magic = "WAT"
                name  = "water"

                [[files]]
                source = "a.txt"
                dest   = "A.TXT"
            "#,
        )
        .write("a.txt", b"a");

    let output = project.pack(&[]);
    assert!(!output.status.success());
    assert!(
        stderr(&output).contains("four ASCII characters"),
        "{}",
        stderr(&output)
    );
}

#[test]
fn a_kernel_role_in_files_is_refused() {
    // Two ways to name the boot image is two ways to name two of them, and
    // the container allows only one.
    let project = Project::new("kernel-role");
    project
        .write(
            "bundle.toml",
            br#"
                magic = "WATR"
                name  = "water"

                [[files]]
                source = "target/kernel7.img"
                dest   = "KERNEL7.IMG"
                role   = "kernel"
            "#,
        )
        .write("target/kernel7.img", b"kernel");

    let output = project.pack(&[]);
    assert!(!output.status.success());
    assert!(
        stderr(&output).contains("[kernel] table"),
        "{}",
        stderr(&output)
    );
}

#[test]
fn the_containers_rules_are_enforced_while_packing() {
    // The point of sharing the crate: this is `rpi-loader-ota` refusing,
    // reported by the packer, rather than a board discovering it later.
    let project = Project::new("unpaired");
    project
        .write(
            "bundle.toml",
            br#"
                magic = "WATR"
                name  = "water"

                [[files]]
                source = "vendor/start.elf"
                dest   = "START.ELF"
                role   = "firmware"
            "#,
        )
        .write("vendor/start.elf", b"start");

    let output = project.pack(&[]);
    assert!(!output.status.success());
    assert!(
        stderr(&output).contains("firmware file without its pair"),
        "{}",
        stderr(&output)
    );
}

#[test]
fn an_unknown_manifest_key_is_refused() {
    // Silently ignoring one means a file a project believed it was
    // shipping is simply absent from every update.
    let project = Project::new("unknown-key");
    project
        .write(
            "bundle.toml",
            br#"
                magic = "WATR"
                name  = "water"
                sauce = "extra"
            "#,
        )
        .write("a.txt", b"a");

    let output = project.pack(&[]);
    assert!(!output.status.success());
    assert!(stderr(&output).contains("sauce"), "{}", stderr(&output));
}

#[test]
fn a_missing_source_names_itself() {
    let project = Project::new("missing-source");
    project.write(
        "bundle.toml",
        br#"
            magic = "WATR"
            name  = "water"

            [kernel]
            source = "target/kernel7.img"
        "#,
    );

    let output = project.pack(&[]);
    assert!(!output.status.success());
    assert!(
        stderr(&output).contains("kernel7.img"),
        "{}",
        stderr(&output)
    );
}
