//! A boot logo baked into the kernel image, and the host-side encoder
//! that puts it there.
//!
//! The image ships as run-length-encoded RGB565 rather than as the PNG it
//! was drawn as, or as the XRGB8888 the framebuffer wants:
//!
//! - **the PNG** needs a zlib inflate at boot, which is a dependency and a
//!   heap allocation to save nothing, since a splash is drawn once.
//! - **raw XRGB8888** needs no decoder at all, but is several times the
//!   size of the rest of a typical kernel image — and the kernel image is
//!   what a UART upload or an over-the-air update has to carry.
//! - **RLE'd RGB565** is smaller than the PNG for a logo (most of one is
//!   flat background) *and* trivially decodable: expanding a run is a
//!   shift and an OR, with no allocation. 5-6-5 costs color precision,
//!   which a logo of flat colors survives.
//!
//! The encoder runs in the board's `build.rs`, behind the `splash-build`
//! feature, which needs `std` and so is a build-dependency only:
//!
//! ```toml
//! [dependencies]
//! rpi-kickstart = { version = "…", features = ["splash"] }
//!
//! [build-dependencies]
//! rpi-kickstart = { version = "…", features = ["splash-build"] }
//! ```
//!
//! ```ignore
//! // build.rs
//! let out = PathBuf::from(env::var("OUT_DIR").unwrap()).join("logo.rle");
//! println!("cargo:rerun-if-changed=assets/logo.png");
//! rpi_kickstart::splash::build::convert("assets/logo.png".as_ref(), &out, Crop::Black)
//!     .unwrap_or_else(|e| panic!("{e}"));
//! ```
//!
//! and the kernel includes what it wrote:
//!
//! ```ignore
//! static LOGO: splash::Image<'static> =
//!     splash::Image::new(include_bytes!(concat!(env!("OUT_DIR"), "/logo.rle")));
//! ```
//!
//! [`Image::new`](crate::splash::Image::new) is a `const fn` that checks the bytes, so a `static`
//! like that one makes a stale or foreign file a compile error rather than
//! a garbled screen.
//!
//! There is no framebuffer here. [`Image::pixels`](crate::splash::Image::pixels) yields the pixels in
//! order and the board blits them wherever its display code puts things.
//!
//! # The format
//!
//! A 12-byte header — [`MAGIC`](crate::splash::MAGIC), then the width and the height as
//! little-endian `u32`s — followed by one `[count, lo, hi]` triple per run
//! of identical RGB565 pixels, `count` from 1 to 255, in left-to-right,
//! top-to-bottom order.

#[cfg(feature = "splash-build")]
pub mod build;

/// The four bytes an encoded image starts with.
pub const MAGIC: [u8; 4] = *b"RLE5";

/// The header's length: [`MAGIC`], the width and the height.
const HEADER: usize = 12;

/// An encoded image, checked.
#[derive(Clone, Copy, Debug)]
pub struct Image<'a> {
    width: usize,
    height: usize,
    runs: &'a [u8],
}

impl<'a> Image<'a> {
    /// Reads `bytes` as an encoded image.
    ///
    /// # Panics
    ///
    /// If `bytes` is not one: no [`MAGIC`], a run section that is not
    /// whole triples, a zero-length run, or runs that do not add up to
    /// exactly `width * height` pixels. In a `static` that panic is a
    /// compile error, which is the intended use.
    pub const fn new(bytes: &'a [u8]) -> Self {
        assert!(
            bytes.len() >= HEADER,
            "splash image: shorter than its header"
        );
        let mut i = 0;
        while i < MAGIC.len() {
            assert!(bytes[i] == MAGIC[i], "splash image: no RLE5 magic");
            i += 1;
        }
        let width = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]) as usize;
        let height = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]) as usize;

        let (_, runs) = bytes.split_at(HEADER);
        assert!(
            runs.len() % 3 == 0,
            "splash image: runs are not whole triples"
        );
        let mut total = 0;
        let mut i = 0;
        while i < runs.len() {
            assert!(runs[i] != 0, "splash image: a run of zero pixels");
            total += runs[i] as usize;
            i += 3;
        }
        assert!(
            total == width * height,
            "splash image: runs do not cover width * height"
        );

        Self {
            width,
            height,
            runs,
        }
    }

    /// The image's width, in pixels.
    pub const fn width(&self) -> usize {
        self.width
    }

    /// The image's height, in pixels.
    pub const fn height(&self) -> usize {
        self.height
    }

    /// The pixels in `0x00RRGGBB` form, left to right then top to bottom —
    /// exactly `width * height` of them.
    pub fn pixels(&self) -> impl Iterator<Item = u32> + 'a {
        self.runs
            .as_chunks::<3>()
            .0
            .iter()
            .flat_map(|&[count, lo, hi]| {
                core::iter::repeat_n(expand(u16::from_le_bytes([lo, hi])), usize::from(count))
            })
    }
}

/// Widens one RGB565 pixel to `0x00RRGGBB`.
///
/// Each channel's high bits are replicated into the low ones it lost,
/// rather than shifted up and zero-filled: that keeps a saturated channel
/// saturated, so pure black and pure white survive the round trip exactly
/// instead of coming back as `0xF8` grays.
const fn expand(pixel: u16) -> u32 {
    let (r, g, b) = ((pixel >> 11) & 0x1F, (pixel >> 5) & 0x3F, pixel & 0x1F);
    let r = (r << 3) | (r >> 2);
    let g = (g << 2) | (g >> 4);
    let b = (b << 3) | (b >> 2);
    ((r as u32) << 16) | ((g as u32) << 8) | b as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Header for a `width` × `height` image, then `runs`.
    fn image(width: u32, height: u32, runs: &[u8]) -> alloc::vec::Vec<u8> {
        let mut bytes = MAGIC.to_vec();
        bytes.extend_from_slice(&width.to_le_bytes());
        bytes.extend_from_slice(&height.to_le_bytes());
        bytes.extend_from_slice(runs);
        bytes
    }

    #[test]
    fn expands_runs_in_order() {
        // Two white, then one pure red, in a 3×1 image.
        let bytes = image(3, 1, &[2, 0xFF, 0xFF, 1, 0x00, 0xF8]);
        let image = Image::new(&bytes);
        assert_eq!((image.width(), image.height()), (3, 1));
        let pixels: alloc::vec::Vec<u32> = image.pixels().collect();
        assert_eq!(pixels, [0xFF_FFFF, 0xFF_FFFF, 0xFF_0000]);
    }

    #[test]
    fn expand_keeps_saturated_channels_saturated() {
        assert_eq!(expand(0x0000), 0x00_0000);
        assert_eq!(expand(0xFFFF), 0xFF_FFFF);
        assert_eq!(expand(0x07E0), 0x00_FF00);
        assert_eq!(expand(0x001F), 0x00_00FF);
    }

    #[test]
    #[should_panic(expected = "no RLE5 magic")]
    fn refuses_a_file_without_the_magic() {
        let mut bytes = image(1, 1, &[1, 0, 0]);
        bytes[0] = b'P';
        Image::new(&bytes);
    }

    #[test]
    #[should_panic(expected = "do not cover")]
    fn refuses_runs_that_do_not_cover_the_image() {
        Image::new(&image(2, 2, &[3, 0, 0]));
    }

    #[test]
    #[should_panic(expected = "whole triples")]
    fn refuses_a_partial_triple() {
        Image::new(&image(1, 1, &[1, 0, 0, 1]));
    }

    #[test]
    #[should_panic(expected = "zero pixels")]
    fn refuses_a_zero_length_run() {
        Image::new(&image(1, 1, &[0, 0, 0, 1, 0, 0]));
    }
}
