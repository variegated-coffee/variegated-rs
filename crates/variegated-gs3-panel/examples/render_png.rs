//! Render every panel state and variant to PNG, for holding against section 6's figures.
//!
//! ```text
//! cargo run --target aarch64-apple-darwin -p variegated-gs3-panel \
//!     --features fixtures --example render_png
//! ```
//!
//! Writes `target/panel/<name>.png` at 1x -- what the panel emits, pixel for pixel -- and
//! `<name>@3x.png`, which is the same image nearest-neighbour scaled so a 8 px label can be
//! read on a monitor. Nothing is anti-aliased at either size, because nothing on the panel
//! is.
//!
//! The dashed rectangle marks the 390x115 the bezel leaves visible. It is drawn *after* the
//! panel and only into the scaled copy, so the 1x image stays exactly what the hardware
//! would show.
//!
//! `png` rather than `embedded-graphics-simulator`: the simulator pulls SDL, which would
//! make the one verification step this crate exists for depend on a system package.

use std::fs::{self, File};
use std::io::BufWriter;
use std::path::Path;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use variegated_gs3_panel::geometry::{PANEL_SIZE, WINDOW_ORIGIN, WINDOW_SIZE};
use variegated_gs3_panel::{fixtures, render};

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

/// The 1x image: exactly what the panel emits.
fn save_1x(fb: &Framebuffer, path: &Path) {
    let mut data = Vec::with_capacity(WIDTH * HEIGHT * 3);
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            data.extend_from_slice(&fb.rgb888(x, y));
        }
    }
    write_png(path, WIDTH, HEIGHT, &data);
}

/// The 3x image, with the visible window outlined so the bezel's edge is obvious.
fn save_3x(fb: &Framebuffer, path: &Path) {
    let (w, h) = (WIDTH * SCALE, HEIGHT * SCALE);
    let mut data = vec![0u8; w * h * 3];
    for y in 0..h {
        for x in 0..w {
            let px = fb.rgb888(x / SCALE, y / SCALE);
            let i = (y * w + x) * 3;
            data[i..i + 3].copy_from_slice(&px);
        }
    }

    // A dashed outline of the window, in the hairline colour, drawn only here.
    let outline = [0x40u8, 0x50, 0x58];
    let (ox, oy) = (WINDOW_ORIGIN.x as usize * SCALE, WINDOW_ORIGIN.y as usize * SCALE);
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
    let out = Path::new("target/panel");
    fs::create_dir_all(out).expect("create target/panel");

    let trace = fixtures::lever_like_trace();
    let aborted = fixtures::lever_like_trace_to(12.4);
    let mut count = 0;

    let emit = |name: &str, view: &variegated_gs3_panel::PanelView<'_>| {
        let mut fb = Framebuffer::new();
        render(view, &mut fb).expect("render");
        save_1x(&fb, &out.join(format!("{name}.png")));
        save_3x(&fb, &out.join(format!("{name}@3x.png")));
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
