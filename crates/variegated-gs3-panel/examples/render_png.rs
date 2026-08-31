//! Render every panel state and variant to PNG, for holding against section 6's figures.
//!
//! ```text
//! cargo run --target aarch64-apple-darwin -p variegated-gs3-panel \
//!     --features fixtures --example render_png
//! ```
//!
//! Writes `target/panel/<name>.png` at 1x -- what the panel emits, pixel for pixel -- and
//! `<name>@3x.png`, the same image nearest-neighbour scaled. Nothing is anti-aliased at
//! either size, because nothing on the panel is.
//!
//! # Both sit on a cream field, and that is not decoration
//!
//! The first round of these panels was approved on a dark page, and three of the four
//! legibility findings that came back off the machine were flattered by it. The bezel is a
//! bright printed ring around a small black window: the eye adapts to the surround, and dim
//! pixels give up contrast they kept on a monitor. Black is the right *surface* -- the panel
//! is emissive and the bezel sits beside the pixels, not behind them -- so what was wrong was
//! the page, not the background.
//!
//! **Judge at 1x.** The same round was approved at 1.5x, which makes an 8 px word look like a
//! 12 px one. The 3x copy is for reading a specific glyph, not for deciding whether it reads.
//!
//! The dashed rectangle marks the 390x115 the bezel leaves visible. It is drawn only into the
//! scaled copy, so the 1x image stays exactly what the hardware would show.
//!
//! `png` rather than `embedded-graphics-simulator`: the simulator pulls SDL, which would
//! make the one verification step this crate exists for depend on a system package.

use std::fs::{self, File};
use std::io::BufWriter;
use std::path::Path;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use variegated_gs3_panel::geometry::{PANEL_SIZE, WINDOW_SIZE};
use variegated_gs3_panel::{Window, fixtures, render};

const WIDTH: usize = PANEL_SIZE.width as usize;
const HEIGHT: usize = PANEL_SIZE.height as usize;
const SCALE: usize = 3;

/// A whole panel in memory, which is all a `DrawTarget` has to be.
struct Framebuffer {
    pixels: Vec<Rgb565>,
}

impl Framebuffer {
    fn new() -> Self {
        Self {
            pixels: vec![Rgb565::BLACK; WIDTH * HEIGHT],
        }
    }

    /// 8-8-8 for the encoder. The panel's own 5-6-5 values are expanded by replicating the
    /// high bits, which is what a display controller does, so the image is what the panel
    /// looks like rather than a darker version of it.
    fn rgb888(&self, x: usize, y: usize) -> [u8; 3] {
        let p = self.pixels[y * WIDTH + x];
        [
            (p.r() << 3) | (p.r() >> 2),
            (p.g() << 2) | (p.g() >> 4),
            (p.b() << 3) | (p.b() >> 2),
        ]
    }
}

impl OriginDimensions for Framebuffer {
    fn size(&self) -> Size {
        PANEL_SIZE
    }
}

impl DrawTarget for Framebuffer {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            // Clipped rather than asserted: this is a preview, and a panel that drew one
            // pixel off the edge should still produce an image showing everything else.
            // `tests/layout.rs` is what fails on an escape.
            if (0..WIDTH as i32).contains(&point.x) && (0..HEIGHT as i32).contains(&point.y) {
                self.pixels[point.y as usize * WIDTH + point.x as usize] = color;
            }
        }
        Ok(())
    }
}

fn write_png(path: &Path, width: usize, height: usize, data: &[u8]) {
    let file = File::create(path).expect("create png");
    let mut encoder = png::Encoder::new(BufWriter::new(file), width as u32, height as u32);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().expect("png header");
    writer.write_image_data(data).expect("png data");
}

/// The bezel's cream, which is what the panel is actually looked at against.
const SURROUND: [u8; 3] = [0xF4, 0xF1, 0xE6];

/// How much of it to show around the panel, in panel pixels.
const MARGIN: usize = 26;

/// The 1x image: exactly what the panel emits, on the surround it is read against.
fn save_1x(fb: &Framebuffer, path: &Path) {
    let (w, h) = (WIDTH + MARGIN * 2, HEIGHT + MARGIN * 2);
    let mut data = Vec::with_capacity(w * h * 3);
    for y in 0..h {
        for x in 0..w {
            if x >= MARGIN && x < MARGIN + WIDTH && y >= MARGIN && y < MARGIN + HEIGHT {
                data.extend_from_slice(&fb.rgb888(x - MARGIN, y - MARGIN));
            } else {
                data.extend_from_slice(&SURROUND);
            }
        }
    }
    write_png(path, w, h, &data);
}

/// The 3x image, with the visible window outlined so the bezel's edge is obvious.
fn save_3x(fb: &Framebuffer, window: Window, path: &Path) {
    let margin = MARGIN * SCALE;
    let (w, h) = (WIDTH * SCALE + margin * 2, HEIGHT * SCALE + margin * 2);
    let mut data = Vec::with_capacity(w * h * 3);
    for y in 0..h {
        for x in 0..w {
            if x >= margin && x < margin + WIDTH * SCALE && y >= margin && y < margin + HEIGHT * SCALE
            {
                data.extend_from_slice(&fb.rgb888((x - margin) / SCALE, (y - margin) / SCALE));
            } else {
                data.extend_from_slice(&SURROUND);
            }
        }
    }

    // A dashed outline of the window, in the hairline colour, drawn only here.
    let outline = [0x40u8, 0x50, 0x58];
    let origin = window.origin();
    let (ox, oy) = (
        margin + origin.x as usize * SCALE,
        margin + origin.y as usize * SCALE,
    );
    let (ww, wh) = (
        WINDOW_SIZE.width as usize * SCALE,
        WINDOW_SIZE.height as usize * SCALE,
    );
    let put = |x: usize, y: usize, data: &mut Vec<u8>| {
        if x < w && y < h {
            let i = (y * w + x) * 3;
            data[i..i + 3].copy_from_slice(&outline);
        }
    };
    for x in (ox..ox + ww).step_by(2) {
        put(x, oy, &mut data);
        put(x, oy + wh - 1, &mut data);
    }
    for y in (oy..oy + wh).step_by(2) {
        put(ox, y, &mut data);
        put(ox + ww - 1, y, &mut data);
    }

    write_png(path, w, h, &data);
}

fn main() {
    // Resolved from the manifest rather than the working directory. `cargo run` inherits the
    // caller's cwd, so a relative path here writes into whichever directory the command was
    // typed in -- which silently produced a second, stale set of these images once.
    let out = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../target/panel")
        .canonicalize()
        .unwrap_or_else(|_| {
            let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/panel");
            fs::create_dir_all(&path).expect("create target/panel");
            path.canonicalize().expect("canonicalize target/panel")
        });
    let out = out.as_path();
    fs::create_dir_all(out).expect("create target/panel");

    let trace = fixtures::lever_like_trace();
    let aborted = fixtures::lever_like_trace_to(12.4);
    let mut count = 0;

    // The default window. A trimmed machine draws the same panels somewhere else, and
    // `shifting_the_window_shifts_every_pixel_with_it` is what asserts that; there is nothing
    // to see in a second set of images of the same content moved seven pixels.
    let window = Window::DEFAULT;

    let emit = |name: &str, view: &variegated_gs3_panel::PanelView<'_>| {
        let mut fb = Framebuffer::new();
        render(view, window, &mut fb).expect("render");
        save_1x(&fb, &out.join(format!("{name}.png")));
        save_3x(&fb, window, &out.join(format!("{name}@3x.png")));
    };

    for (name, view) in fixtures::all(&trace, &aborted) {
        emit(name, &view);
        count += 1;
    }
    for (name, view) in fixtures::overlays_only() {
        emit(name, &view);
        count += 1;
    }

    // The one thing a rendered PNG cannot state, so it is stated here: the curve is drawn
    // from a trace this many buckets wide, which is what the folding produced for a 51 s
    // shot.
    println!(
        "{count} panels written to {} (curve: {} buckets at {} ms)",
        out.display(),
        trace.pressure().len(),
        trace.bucket_ms(),
    );
}
