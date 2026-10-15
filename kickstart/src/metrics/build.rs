//! The capture: what [`super::BuildInfo`] carries, read on the host.
//!
//! For a board's `build.rs`, so it needs `std` and runs on the host; see
//! the parent module for the manifest lines. Unlike the splash encoder this
//! does print `cargo:` directives, because they are the whole of what it
//! produces: the values go to the board's compile as environment variables,
//! and the build script has to be re-run whenever one of them would change.

use std::path::{Path, PathBuf};
use std::process::Command;
use std::string::{String, ToString};
use std::vec::Vec;
use std::{env, fs, println};

use sha2::{Digest, Sha256};

/// Written for anything that could not be found out: not built from a git
/// checkout, no `git` on the path, no `Cargo.lock` above the manifest.
pub const UNKNOWN: &str = "unknown";

/// What [`emit`] hands the board's compile.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Capture {
    /// [`BuildInfo::git`](super::BuildInfo::git).
    pub git: String,
    /// [`BuildInfo::dirty`](super::BuildInfo::dirty).
    pub dirty: String,
    /// [`BuildInfo::lock`](super::BuildInfo::lock).
    pub lock: String,
    /// The files whose change can change any of the above, for
    /// `rerun-if-changed`.
    pub watch: Vec<PathBuf>,
}

impl Capture {
    /// The `cargo:` lines that hand this to the compile, in the order
    /// [`emit`] prints them.
    ///
    /// The variable names are the ones [`build_info!`](crate::build_info)
    /// reads, which has to spell them as literals.
    pub fn directives(&self) -> Vec<String> {
        let mut lines = Vec::with_capacity(3 + self.watch.len());
        lines.push(std::format!(
            "cargo:rustc-env=KICKSTART_BUILD_GIT={}",
            self.git
        ));
        lines.push(std::format!(
            "cargo:rustc-env=KICKSTART_BUILD_DIRTY={}",
            self.dirty
        ));
        lines.push(std::format!(
            "cargo:rustc-env=KICKSTART_BUILD_LOCK={}",
            self.lock
        ));
        for path in &self.watch {
            lines.push(std::format!("cargo:rerun-if-changed={}", path.display()));
        }
        lines
    }
}

/// Captures the board being built and prints the directives that hand it
/// to the compile. The one call a board's `build.rs` makes.
///
/// It names every file it depends on with `rerun-if-changed` — and once a
/// build script names any file, Cargo re-runs it for those alone. A board
/// whose script does other work names that work's inputs too, as it would
/// anyway.
///
/// # Panics
///
/// If `CARGO_MANIFEST_DIR` is unset, which Cargo always sets for a build
/// script — so only when called from somewhere that is not one.
pub fn emit() {
    let manifest = env::var_os("CARGO_MANIFEST_DIR")
        .expect("CARGO_MANIFEST_DIR is unset: call emit() from a build script");
    for line in capture(Path::new(&manifest)).directives() {
        println!("{line}");
    }
}

/// Reads the commit, the tree's state and the lock hash for the crate whose
/// manifest is in `manifest_dir`, without printing anything.
pub fn capture(manifest_dir: &Path) -> Capture {
    let mut watch = Vec::new();

    let lock = match find_lock(manifest_dir) {
        Some(path) => {
            let hash = fs::read(&path).map(|bytes| lock_hash(&bytes));
            watch.push(path);
            hash.unwrap_or_else(|_| UNKNOWN.to_string())
        }
        None => UNKNOWN.to_string(),
    };

    let ask = |args: &[&str]| git(manifest_dir, args);
    let (commit, dirty) = match ask(&["rev-parse", "--short", "HEAD"]) {
        Some(commit) => {
            // `--no-optional-locks` so that asking does not refresh the
            // index: a refresh rewrites `.git/index`, which is watched
            // below, and would make every build re-run this one more time.
            // `--untracked-files=no` because a stray file is not part of
            // what was built, as `git describe --dirty` has it.
            let dirty = match ask(&[
                "--no-optional-locks",
                "status",
                "--porcelain",
                "--untracked-files=no",
            ]) {
                Some(status) => (!status.is_empty()).to_string(),
                None => UNKNOWN.to_string(),
            };
            (commit, dirty)
        }
        None => (UNKNOWN.to_string(), UNKNOWN.to_string()),
    };

    if commit != UNKNOWN {
        watch.extend(git_inputs(manifest_dir));
    }
    // A file named but missing makes Cargo re-run the script on every build,
    // so only what exists is named.
    watch.retain(|path| path.exists());

    Capture {
        git: commit,
        dirty,
        lock,
        watch,
    }
}

/// The first eight hex digits of `bytes`'s SHA-256.
///
/// SHA-256 rather than something cheaper so the value can be checked
/// against a file by hand, with `sha256sum Cargo.lock | cut -c1-8`. Eight
/// digits are for telling builds apart, not for resisting anyone.
pub fn lock_hash(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest[..4]
        .iter()
        .map(|b| std::format!("{b:02x}"))
        .collect()
}

/// The `Cargo.lock` that governs the crate in `manifest_dir`: its own, or,
/// for a workspace member, the nearest one above it.
pub fn find_lock(manifest_dir: &Path) -> Option<PathBuf> {
    manifest_dir
        .ancestors()
        .map(|dir| dir.join("Cargo.lock"))
        .find(|path| path.is_file())
}

/// Every file whose change can move the commit or the dirty flag: `HEAD`,
/// the branch it names, the packed refs, the index — and each tracked
/// file, because an edit that is not yet staged changes none of the others
/// and still makes the tree dirty.
fn git_inputs(dir: &Path) -> Vec<PathBuf> {
    let mut inputs = Vec::new();
    let git_dir = git(dir, &["rev-parse", "--absolute-git-dir"]).map(PathBuf::from);
    // A worktree keeps its own `HEAD` and index but shares the refs.
    let common_dir = git(
        dir,
        &["rev-parse", "--path-format=absolute", "--git-common-dir"],
    )
    .map(PathBuf::from);
    if let Some(git_dir) = &git_dir {
        inputs.push(git_dir.join("HEAD"));
        inputs.push(git_dir.join("index"));
    }
    if let Some(common_dir) = common_dir.as_ref().or(git_dir.as_ref()) {
        inputs.push(common_dir.join("packed-refs"));
        if let Some(branch) = git(dir, &["symbolic-ref", "-q", "HEAD"]) {
            inputs.push(common_dir.join(branch));
        }
    }
    if let Some(top) = git(dir, &["rev-parse", "--show-toplevel"]).map(PathBuf::from)
        && let Some(files) = git(&top, &["ls-files", "-z"])
    {
        inputs.extend(
            files
                .split('\0')
                .filter(|file| !file.is_empty())
                .map(|file| top.join(file)),
        );
    }
    inputs
}

/// `git -C dir args…`'s output, trimmed, or `None` if `git` is missing,
/// fails, or prints something that is not UTF-8.
fn git(dir: &Path, args: &[&str]) -> Option<String> {
    let output = Command::new("git")
        .arg("-C")
        .arg(dir)
        .args(args)
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    String::from_utf8(output.stdout)
        .ok()
        .map(|text| text.trim_end().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_lock_hash_is_sha256s_first_eight_digits() {
        // sha256("") = e3b0c442 98fc1c14 ...
        assert_eq!(lock_hash(b""), "e3b0c442");
        // sha256("abc") = ba7816bf 8f01cfea ...
        assert_eq!(lock_hash(b"abc"), "ba7816bf");
    }

    #[test]
    fn directives_use_the_names_build_info_reads() {
        let capture = Capture {
            git: "a1b2c3d".to_string(),
            dirty: "false".to_string(),
            lock: "9f8e7d6c".to_string(),
            watch: std::vec![PathBuf::from("/x/Cargo.lock")],
        };
        assert_eq!(
            capture.directives(),
            [
                "cargo:rustc-env=KICKSTART_BUILD_GIT=a1b2c3d",
                "cargo:rustc-env=KICKSTART_BUILD_DIRTY=false",
                "cargo:rustc-env=KICKSTART_BUILD_LOCK=9f8e7d6c",
                "cargo:rerun-if-changed=/x/Cargo.lock",
            ]
        );
    }

    /// Outside any checkout and with no lock above it, everything is
    /// `unknown` and nothing is watched. Skipped where the temporary
    /// directory itself sits inside a checkout or under a lock.
    #[test]
    fn nothing_found_is_unknown() {
        let temp = env::temp_dir();
        if find_lock(&temp).is_some() || git(&temp, &["rev-parse"]).is_some() {
            return;
        }
        let dir = temp.join(std::format!("kickstart-build-info-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let capture = capture(&dir);
        fs::remove_dir_all(&dir).unwrap();
        assert_eq!(capture.git, UNKNOWN);
        assert_eq!(capture.dirty, UNKNOWN);
        assert_eq!(capture.lock, UNKNOWN);
        assert!(capture.watch.is_empty());
    }

    /// This crate, built from its own checkout: a commit, a definite dirty
    /// flag, and its own lock hashed and watched.
    #[test]
    fn this_crate_is_captured() {
        let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
        let capture = capture(manifest);
        let lock = manifest.join("Cargo.lock");
        assert_eq!(capture.lock, lock_hash(&fs::read(&lock).unwrap()));
        assert!(capture.watch.contains(&lock));
        if capture.git != UNKNOWN {
            assert!(capture.git.chars().all(|c| c.is_ascii_hexdigit()));
            assert!(capture.dirty == "true" || capture.dirty == "false");
            assert!(capture.watch.contains(&manifest.join("Cargo.toml")));
        }
    }
}
