//! The encoder: a PNG in, the format [`super::Image`] reads out.
//!
//! For a board's `build.rs`, so it needs `std` and runs on the host; see
//! the parent module for the manifest lines. It prints no `cargo:`
//! directives itself — the build script names the file it depends on, so
//! that calling this from anywhere else has no side effects on a build.

use std::fmt;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::vec;
use std::vec::Vec;

use super::MAGIC;

/// What to do with a black margin the artwork was exported with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Crop {
    /// Keep the image whole.
    None,
    /// Drop every whole row and column, from each edge inward, that is
    /// entirely black (`0,0,0`, after compositing).
    ///
    /// A logo exported on a screen-sized canvas arrives with a margin
    /// baked in, and the margin decides where the logo sits: a board
    /// centers what it is given, so the margin is the difference between
    /// centering the artwork and centering a canvas that is only
    /// symmetric if the export happened to be. It is also what gets
    /// clipped first on a screen smaller than the canvas.
    ///
    /// Exactly black rather than nearly so: the test is whether a pixel is
    /// indistinguishable from the background it is drawn onto, and a
    /// threshold would eventually clip something faint that was meant to
    /// be there. An image with nothing but black in it is kept whole.
    Black,
}

/// Why [`convert`] failed.
#[derive(Debug)]
pub enum Error {
    /// The PNG could not be read.
    Read(PathBuf, io::Error),
    /// It is not a PNG the decoder accepts.
    Decode(PathBuf, png::DecodingError),
    /// The decoder produced neither RGB nor RGBA from it.
    Color(PathBuf, png::ColorType),
    /// Its dimensions do not fit the header's `u32`s.
    TooLarge(PathBuf),
    /// The encoded image could not be written.
    Write(PathBuf, io::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Read(path, e) => write!(f, "reading {}: {e}", path.display()),
            Self::Decode(path, e) => write!(f, "decoding {}: {e}", path.display()),
            Self::Color(path, color) => {
                write!(f, "{}: unsupported color type {color:?}", path.display())
            }
            Self::TooLarge(path) => write!(f, "{}: too large for a splash", path.display()),
            Self::Write(path, e) => write!(f, "writing {}: {e}", path.display()),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Read(_, e) | Self::Write(_, e) => Some(e),
            Self::Decode(_, e) => Some(e),
            Self::Color(..) | Self::TooLarge(_) => None,
        }
    }
}

/// Encodes the PNG at `png` into `out`, returning the width and height of
/// what was written.
///
/// Paletted, grayscale and 16-bit sources are normalized to 8-bit color,
/// and alpha is composited over black — what a splash is drawn onto, so a
/// pixel the artwork left transparent and one it painted black are the
/// same pixel on screen. Then `crop`, then the encoding.
pub fn convert(png: &Path, out: &Path, crop: Crop) -> Result<(usize, usize), Error> {
    let (width, height, rgb) = decode_rgb8(png)?;
    let (width, height, rgb) = match crop {
        Crop::None => (width, height, rgb),
        Crop::Black => trim_black(width, height, rgb),
    };
    let bytes = encode(width, height, &rgb).ok_or_else(|| Error::TooLarge(png.to_path_buf()))?;
    fs::write(out, bytes).map_err(|e| Error::Write(out.to_path_buf(), e))?;
    Ok((width, height))
}

/// Encodes `width * height` RGB888 triples, header included, or `None` if
/// a dimension does not fit a `u32`.
pub fn encode(width: usize, height: usize, rgb: &[u8]) -> Option<Vec<u8>> {
    assert_eq!(
        rgb.len(),
        width * height * 3,
        "rgb is not width * height pixels"
    );
    let mut bytes = MAGIC.to_vec();
    bytes.extend_from_slice(&u32::try_from(width).ok()?.to_le_bytes());
    bytes.extend_from_slice(&u32::try_from(height).ok()?.to_le_bytes());
    bytes.extend_from_slice(&encode_rle565(rgb));
    Some(bytes)
}

/// Decodes `path` to 8-bit RGB triples over black, with its dimensions.
fn decode_rgb8(path: &Path) -> Result<(usize, usize, Vec<u8>), Error> {
    let decode = |e| Error::Decode(path.to_path_buf(), e);
    // `Cursor`, not the `File` itself: png's `Decoder` wants `BufRead +
    // Seek`, and a logo is a few hundred KiB at most.
    let file = fs::read(path).map_err(|e| Error::Read(path.to_path_buf(), e))?;
    let mut decoder = png::Decoder::new(io::Cursor::new(file));
    decoder.set_transformations(png::Transformations::normalize_to_color8());
    let mut reader = decoder.read_info().map_err(decode)?;

    let size = reader
        .output_buffer_size()
        .ok_or_else(|| Error::TooLarge(path.to_path_buf()))?;
    let mut buffer = vec![0; size];
    let info = reader.next_frame(&mut buffer).map_err(decode)?;
    let pixels = &buffer[..info.buffer_size()];

    let rgb = match info.color_type {
        png::ColorType::Rgb => pixels.to_vec(),
        png::ColorType::Rgba => over_black(pixels),
        other => return Err(Error::Color(path.to_path_buf(), other)),
    };
    Ok((info.width as usize, info.height as usize, rgb))
}

/// Flattens RGBA onto black, dropping the alpha channel.
fn over_black(rgba: &[u8]) -> Vec<u8> {
    rgba.as_chunks::<4>()
        .0
        .iter()
        .flat_map(|px| {
            let alpha = u32::from(px[3]);
            [0, 1, 2].map(|c| ((u32::from(px[c]) * alpha + 127) / 255) as u8)
        })
        .collect()
}

/// [`Crop::Black`]: the image left once the black margin is gone.
fn trim_black(width: usize, height: usize, rgb: Vec<u8>) -> (usize, usize, Vec<u8>) {
    let lit = |x: usize, y: usize| rgb[(y * width + x) * 3..][..3] != [0, 0, 0];

    let rows = || (0..height).filter(|&y| (0..width).any(|x| lit(x, y)));
    let columns = || (0..width).filter(|&x| (0..height).any(|y| lit(x, y)));

    // Keeping an all-black image whole rather than producing a zero-sized
    // one keeps the failure visible as a black screen instead of as a
    // panic in the build of an unrelated change.
    let (Some(top), Some(bottom)) = (rows().next(), rows().next_back()) else {
        return (width, height, rgb);
    };
    let (left, right) = (columns().next().unwrap(), columns().next_back().unwrap());

    let (cropped_width, cropped_height) = (right - left + 1, bottom - top + 1);
    let mut out = Vec::with_capacity(cropped_width * cropped_height * 3);
    for y in top..=bottom {
        out.extend_from_slice(&rgb[(y * width + left) * 3..][..cropped_width * 3]);
    }
    (cropped_width, cropped_height, out)
}

/// Run-length-encodes RGB888 triples as RGB565 ones: `[count, lo, hi]` per
/// run, with `count` capped at 255 so it fits a byte.
fn encode_rle565(rgb: &[u8]) -> Vec<u8> {
    let mut runs: Vec<u8> = Vec::new();
    for pixel in rgb.as_chunks::<3>().0 {
        let [r, g, b] = pixel.map(u16::from);
        let packed = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
        let [lo, hi] = packed.to_le_bytes();

        match runs.last_chunk_mut::<3>() {
            Some([count, l, h]) if *l == lo && *h == hi && *count < u8::MAX => *count += 1,
            _ => runs.extend_from_slice(&[1, lo, hi]),
        }
    }
    runs
}

#[cfg(test)]
mod tests {
    use super::super::Image;
    use super::*;

    #[test]
    fn round_trips_through_image() {
        // 300 white pixels then one red: the white crosses the 255 cap.
        let mut rgb = [0xFF; 301 * 3].to_vec();
        rgb[300 * 3..].copy_from_slice(&[0xFF, 0, 0]);
        let bytes = encode(301, 1, &rgb).unwrap();
        let image = Image::new(&bytes);
        let pixels: Vec<u32> = image.pixels().collect();
        assert_eq!(pixels.len(), 301);
        assert!(pixels[..300].iter().all(|&p| p == 0xFF_FFFF));
        assert_eq!(pixels[300], 0xFF_0000);
        // 255 + 45 white, then the red.
        assert_eq!(bytes.len(), 12 + 3 * 3);
    }

    #[test]
    fn trims_only_the_black_margin() {
        // 4×3, one lit pixel at (1, 1) and one at (2, 1).
        let mut rgb = vec![0; 4 * 3 * 3];
        rgb[(4 + 1) * 3] = 9;
        rgb[(4 + 2) * 3 + 2] = 9;
        let (w, h, out) = trim_black(4, 3, rgb);
        assert_eq!((w, h), (2, 1));
        assert_eq!(out, [9, 0, 0, 0, 0, 9]);
    }

    #[test]
    fn keeps_an_all_black_image_whole() {
        let (w, h, out) = trim_black(2, 2, vec![0; 12]);
        assert_eq!((w, h, out.len()), (2, 2, 12));
    }

    #[test]
    fn composites_alpha_over_black() {
        assert_eq!(
            over_black(&[200, 100, 50, 0, 200, 100, 50, 255]),
            [0, 0, 0, 200, 100, 50]
        );
    }
}
