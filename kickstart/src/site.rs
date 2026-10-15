//! A board's web assets, read off the card at boot and served from RAM.
//!
//! Everything under `/WWW` is loaded onto the heap during bring-up and kept
//! there. Serving a page then costs a lookup rather than a card read, which
//! matters because a card behind a blocking driver would otherwise stall
//! the executor — and with it the network — once per file on every page
//! load. It is affordable because a Pi's heap is hundreds of megabytes; on
//! a microcontroller the same design would be a per-request read.
//!
//! ```ignore
//! static SITE: StaticCell<Site> = StaticCell::new();
//! let site: &'static Site = SITE.init(site::load(&mut volume)?);
//! logln!("site: {} files", site.len());
//! // later, in a request handler:
//! if let Some(asset) = site.get(path) { /* serve asset.body as asset.content_type */ }
//! ```
//!
//! # Why the site is on the card
//!
//! So that it is edited as files. A stylesheet is a stylesheet and a page's
//! script is a module a browser caches, rather than all of it being one
//! string in the kernel that has to be rebuilt and reflashed to change a
//! label. An over-the-air update can replace it like any other file.
//!
//! The tree is walked, not just the top directory, and files keep the
//! names they were written with rather than an 8.3 alias: `resident-fat`
//! hands back the long name when an entry has one. So a request path maps
//! to a card path directly, and the site can be laid out in directories.

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;

use resident_fat::{BlockDevice, DateTime, Error};

use crate::logln;
use crate::storage::Volume;

/// Directory on the card holding the web assets.
pub const WWW_DIR: &str = "WWW";

/// The page served for `/`, and for any path ending in `/`.
const INDEX: &str = "index.html";

/// How many directory levels below [`WWW_DIR`] are walked.
///
/// A bound rather than an open recursion. A site nests nowhere near this
/// deep, so hitting it means the card holds something unexpected — and this
/// runs before the network is up, where a walk that never finished would
/// look like a board that failed to boot.
const MAX_DEPTH: usize = 4;

/// Content types by extension, for the files a small status site is made
/// of. Anything not here — and not in a board's own table, see
/// [`load_with`] — is served as `application/octet-stream`, which a browser
/// downloads rather than renders: visible enough to notice, harmless if it
/// happens.
///
/// `text/javascript` matters more than the rest. A browser refuses to run
/// a module script served as anything else, so getting `.js` wrong is a
/// blank page rather than a downloaded file.
const CONTENT_TYPES: &[(&str, &str)] = &[
    ("html", "text/html; charset=utf-8"),
    ("htm", "text/html; charset=utf-8"),
    ("css", "text/css; charset=utf-8"),
    ("js", "text/javascript; charset=utf-8"),
    ("json", "application/json"),
    ("svg", "image/svg+xml"),
    ("ico", "image/x-icon"),
    ("png", "image/png"),
    ("txt", "text/plain; charset=utf-8"),
];

/// Served when no table names the extension.
const FALLBACK_TYPE: &str = "application/octet-stream";

/// One file held in memory.
#[derive(Debug)]
pub struct Asset {
    /// Path relative to [`WWW_DIR`], `/`-separated and in the case the card
    /// stores — `js/pages/system.js`, not `JS/PAGES/SYSTEM.JS`.
    pub path: String,
    /// File contents.
    pub body: Vec<u8>,
    /// MIME type, from the extension.
    pub content_type: &'static str,
}

/// Every asset under [`WWW_DIR`], as [`load`] found them.
///
/// Owned rather than installed in a static, so where it lives is the
/// board's choice; a web server wants it `'static`, which a `StaticCell`
/// gives.
#[derive(Debug, Default)]
pub struct Site {
    assets: Vec<Asset>,
}

impl Site {
    /// Looks an asset up by request path.
    ///
    /// `/`, and any path ending in `/`, maps to that directory's
    /// `index.html`. Otherwise the leading slash is stripped and the rest
    /// matched case-insensitively, because FAT matches names that way and
    /// a link written in the wrong case should not be the difference
    /// between a page and a 404.
    ///
    /// The path is matched as given: pass it without the query string, and
    /// note that percent-escapes are not decoded, so a file whose name
    /// needs one is not reachable. No traversal check is needed and none
    /// is made — this resolves against a list built at boot, not against
    /// the card, so a path full of `..` can only fail to match.
    pub fn get(&self, path: &str) -> Option<&Asset> {
        let path = path.strip_prefix('/').unwrap_or(path);
        let index;
        let wanted = if path.is_empty() || path.ends_with('/') {
            index = alloc::format!("{path}{INDEX}");
            index.as_str()
        } else {
            path
        };
        self.assets
            .iter()
            .find(|asset| asset.path.eq_ignore_ascii_case(wanted))
    }

    /// How many files were loaded.
    pub fn len(&self) -> usize {
        self.assets.len()
    }

    /// Whether no files were loaded — no `/WWW`, or an empty one.
    pub fn is_empty(&self) -> bool {
        self.assets.is_empty()
    }

    /// Every asset, in the order they were loaded.
    pub fn iter(&self) -> impl Iterator<Item = &Asset> {
        self.assets.iter()
    }
}

/// Loads everything under [`WWW_DIR`] with the built-in content types.
/// See [`load_with`].
pub fn load<D: BlockDevice>(volume: &mut Volume<D>) -> Result<Site, Error<D::Error>> {
    load_with(volume, &[])
}

/// Loads everything under [`WWW_DIR`], logging each file with its size and
/// modification time.
///
/// `content_types` is `(extension, type)` pairs consulted before the
/// built-in table, so a board serving something exotic — `wasm`, `webp` —
/// or disagreeing about a common one does not need a fork. Extensions are
/// without the dot and matched case-insensitively.
///
/// **A missing directory is not an error.** It is an empty [`Site`] and a
/// console line: a board's job is rarely its web page, and it must still do
/// that job with no assets at all — an update can supply them later. A
/// directory that exists and cannot be read is an error, and whether to
/// boot without a site is the caller's decision.
///
/// Call it during bring-up, while the card is nobody else's.
pub fn load_with<D: BlockDevice>(
    volume: &mut Volume<D>,
    content_types: &[(&str, &'static str)],
) -> Result<Site, Error<D::Error>> {
    let mut assets = Vec::new();
    match read_all(volume, content_types, &mut assets) {
        Ok(()) => {}
        Err(Error::NotFound { name })
            if assets.is_empty() && name.eq_ignore_ascii_case(WWW_DIR) =>
        {
            logln!("site: no /{WWW_DIR} on the card; serving no pages");
        }
        Err(error) => return Err(error),
    }
    Ok(Site { assets })
}

/// A file's timestamp, printed.
///
/// `no timestamp` is the FAT epoch, which is what a file carries when it
/// was written by a board that did not know the time — or by `resident-fat`
/// before anything gave it a clock. Anything else is a real clock having
/// reached the card, which is how an update that did replace the site is
/// told from one that did not without pulling the card.
struct Stamp(DateTime);

impl fmt::Display for Stamp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let at = &self.0;
        if *at == DateTime::EPOCH {
            return f.write_str("no timestamp");
        }
        write!(
            f,
            "{:04}-{:02}-{:02} {:02}:{:02}",
            at.year, at.month, at.day, at.hour, at.minute
        )
    }
}

/// Reads every file under [`WWW_DIR`], at any depth up to [`MAX_DEPTH`],
/// into `out`.
///
/// A worklist rather than recursion: opening a file needs the volume back,
/// so a directory's borrow has to end before any of its files can be read,
/// and a recursive walk would be holding one open at every level.
fn read_all<D: BlockDevice>(
    volume: &mut Volume<D>,
    content_types: &[(&str, &'static str)],
    out: &mut Vec<Asset>,
) -> Result<(), Error<D::Error>> {
    // Directories still to visit, relative to `WWW_DIR`; the root is "".
    let mut pending: Vec<(String, usize)> = alloc::vec![(String::new(), 0)];

    while let Some((directory, depth)) = pending.pop() {
        let absolute = if directory.is_empty() {
            String::from(WWW_DIR)
        } else {
            alloc::format!("{WWW_DIR}/{directory}")
        };

        // Names first, then contents: the directory is borrowed from the
        // volume, and opening a file needs it back.
        let mut files: Vec<(String, DateTime)> = Vec::new();
        for entry in volume.open_dir(&absolute)?.iter() {
            let name = entry.name();
            // `.` and `..` are ordinary entries on FAT, and a walk that
            // followed them would not end.
            if name == "." || name == ".." {
                continue;
            }
            let path = if directory.is_empty() {
                String::from(name)
            } else {
                alloc::format!("{directory}/{name}")
            };
            if entry.is_directory() {
                if depth + 1 > MAX_DEPTH {
                    logln!("site: /{WWW_DIR}/{path} is deeper than {MAX_DEPTH}; not walked");
                    continue;
                }
                pending.push((path, depth + 1));
            } else {
                files.push((path, entry.modified()));
            }
        }

        for (path, modified) in files {
            let file = volume.open(&alloc::format!("{WWW_DIR}/{path}"))?;
            // One read for the whole file, which for a contiguous one is a
            // single card transfer however large it is.
            let body = volume.read_all(&file)?;
            logln!(
                "site: /{WWW_DIR}/{path} ({} bytes, {})",
                body.len(),
                Stamp(modified)
            );
            let content_type = content_type_for(&path, content_types);
            out.push(Asset {
                path,
                body,
                content_type,
            });
        }
    }
    Ok(())
}

/// The content type for `path`, from its extension: `overrides` first, then
/// [`CONTENT_TYPES`], then [`FALLBACK_TYPE`].
///
/// The extension is what follows the last `.` in the file name — not in
/// the whole path, so a directory with a dot in its name does not lend its
/// file an extension. A name with no dot, or only a leading one
/// (`.htaccess`), has none.
fn content_type_for(path: &str, overrides: &[(&str, &'static str)]) -> &'static str {
    let name = path.rsplit('/').next().unwrap_or(path);
    let Some((stem, extension)) = name.rsplit_once('.') else {
        return FALLBACK_TYPE;
    };
    if stem.is_empty() {
        return FALLBACK_TYPE;
    }
    overrides
        .iter()
        .chain(CONTENT_TYPES)
        .find(|(known, _)| known.eq_ignore_ascii_case(extension))
        .map_or(FALLBACK_TYPE, |&(_, content_type)| content_type)
}

#[cfg(test)]
mod tests {
    use alloc::vec;

    use super::*;

    fn site(paths: &[&str]) -> Site {
        Site {
            assets: paths
                .iter()
                .map(|path| Asset {
                    path: String::from(*path),
                    body: Vec::new(),
                    content_type: content_type_for(path, &[]),
                })
                .collect(),
        }
    }

    #[test]
    fn root_and_trailing_slash_are_the_index() {
        let site = site(&["index.html", "docs/index.html", "app.js"]);
        assert_eq!(site.get("/").unwrap().path, "index.html");
        assert_eq!(site.get("").unwrap().path, "index.html");
        assert_eq!(site.get("/docs/").unwrap().path, "docs/index.html");
        assert!(site.get("/docs").is_none());
    }

    #[test]
    fn lookup_is_case_insensitive_and_exact() {
        let site = site(&["js/pages/System.js"]);
        assert!(site.get("/js/pages/system.js").is_some());
        assert!(site.get("/JS/PAGES/SYSTEM.JS").is_some());
        assert!(site.get("/js/pages/system").is_none());
        assert!(site.get("/js/pages/../pages/system.js").is_none());
        assert!(site.get("/pages/system.js").is_none());
    }

    #[test]
    fn an_empty_site_finds_nothing() {
        let site = Site::default();
        assert!(site.is_empty());
        assert!(site.get("/").is_none());
    }

    #[test]
    fn content_types() {
        assert_eq!(
            content_type_for("index.html", &[]),
            "text/html; charset=utf-8"
        );
        assert_eq!(
            content_type_for("a/B.JS", &[]),
            "text/javascript; charset=utf-8"
        );
        assert_eq!(content_type_for("font.woff2", &[]), FALLBACK_TYPE);
        assert_eq!(content_type_for("README", &[]), FALLBACK_TYPE);
        assert_eq!(content_type_for(".htaccess", &[]), FALLBACK_TYPE);
        // The directory's dot is not the file's extension.
        assert_eq!(content_type_for("v1.2/LICENSE", &[]), FALLBACK_TYPE);
    }

    #[test]
    fn a_boards_table_comes_first() {
        let table = vec![
            ("wasm", "application/wasm"),
            ("js", "application/javascript"),
        ];
        assert_eq!(content_type_for("app.WASM", &table), "application/wasm");
        assert_eq!(content_type_for("app.js", &table), "application/javascript");
        assert_eq!(
            content_type_for("app.css", &table),
            "text/css; charset=utf-8"
        );
    }

    #[test]
    fn stamps() {
        assert_eq!(alloc::format!("{}", Stamp(DateTime::EPOCH)), "no timestamp");
        let mut at = DateTime::EPOCH;
        at.year = 2026;
        at.month = 9;
        at.day = 27;
        at.hour = 14;
        at.minute = 5;
        assert_eq!(alloc::format!("{}", Stamp(at)), "2026-09-27 14:05");
    }
}
