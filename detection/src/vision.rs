//! # Vision Processing & Depth Extraction Module
//!
//! Provides PNG depth decoding, region-of-interest horizon filtering, centroid offset calculation,
//! and fallback asset generation for offline sensor streams.

use image::{ImageBuffer, ImageReader, Luma, Rgb, RgbImage};
use serde::Serialize;
use std::io::Cursor;

// =============================================================================
// DOMAIN STRUCTURES
// =============================================================================

/// Container for processed obstacle detection telemetry dispatched over UDP to `movement`.
#[derive(Serialize, Debug, Clone, Copy)]
pub struct ObstacleTelemetry {
    /// True if a valid obstacle is detected within spatial thresholds.
    pub obstacle_detected: bool,
    /// Distance to obstacle centroid in meters.
    pub distance_m: f64,
    /// Bearing angle to obstacle centroid in degrees (-30.0 to +30.0).
    pub angle_deg: f64,
}

// =============================================================================
// VISION PROCESSING PIPELINE
// =============================================================================

/// Decodes raw 16-bit PNG bytes into a Luma16 image buffer.
pub fn decode_depth_png(
    png_bytes: &[u8],
) -> Result<ImageBuffer<Luma<u16>, Vec<u16>>, Box<dyn std::error::Error>> {
    let image = ImageReader::new(Cursor::new(png_bytes))
        .with_guessed_format()?
        .decode()?;
    Ok(image.to_luma16())
}

/// Evaluates RGB and 16-bit depth streams, applying region-of-interest horizon filters
/// to derive spatial obstacle telemetry while ignoring ground-plane clutter.
pub fn process_sensor_streams(
    _jpeg_bytes: &[u8],
    depth_png_bytes: &[u8],
) -> Result<ObstacleTelemetry, Box<dyn std::error::Error>> {
    let depth_image = decode_depth_png(depth_png_bytes)?;
    let (width, height) = depth_image.dimensions();

    // Restrict vertical scan band to 12%-42% height to filter ground plane below camera (z=0.085m).
    let x_start = width * 20 / 100;
    let x_end = width * 80 / 100;
    let y_start = height * 12 / 100;
    let y_end = height * 42 / 100;

    let mut close_pixel_count: u64 = 0;
    let mut sum_depth_mm: u64 = 0;
    let mut sum_x_pixels: u64 = 0;

    let min_dist_mm = 300u16;  // Minimum detection threshold (0.3m)
    let max_dist_mm = 2000u16; // Maximum detection horizon (2.0m)

    for y in y_start..y_end {
        for x in x_start..x_end {
            let depth_mm = depth_image.get_pixel(x, y)[0];
            if depth_mm >= min_dist_mm && depth_mm <= max_dist_mm {
                close_pixel_count += 1;
                sum_depth_mm += depth_mm as u64;
                sum_x_pixels += x as u64;
            }
        }
    }

    if close_pixel_count > 100 {
        let avg_depth_mm = (sum_depth_mm / close_pixel_count) as f64;
        let centroid_x = (sum_x_pixels / close_pixel_count) as f64;
        let center_x = (width / 2) as f64;

        let distance_m = avg_depth_mm / 1000.0;
        let angle_deg = ((centroid_x - center_x) / center_x) * 30.0;

        Ok(ObstacleTelemetry {
            obstacle_detected: true,
            distance_m,
            angle_deg,
        })
    } else {
        Ok(ObstacleTelemetry {
            obstacle_detected: false,
            distance_m: 0.0,
            angle_deg: 0.0,
        })
    }
}

// =============================================================================
// FALLBACK ASSET GENERATOR
// =============================================================================

/// Generates synthetic static fallback image assets (`fallback_rgb.jpg` and `fallback_depth.png`)
/// if disk assets are missing upon system startup.
pub fn create_default_fallback_images<P: AsRef<std::path::Path>>(
    out_dir: P,
) -> Result<(), Box<dyn std::error::Error>> {
    let width = 640u32;
    let height = 480u32;

    let mut rgb_img = RgbImage::new(width, height);
    for x in 270..370 {
        for y in 190..290 {
            rgb_img.put_pixel(x, y, Rgb([255, 0, 0]));
        }
    }
    rgb_img.save(out_dir.as_ref().join("fallback_rgb.jpg"))?;

    let mut depth_img = ImageBuffer::<Luma<u16>, Vec<u16>>::new(width, height);
    for x in 0..width {
        for y in 0..height {
            depth_img.put_pixel(x, y, Luma([1500]));
        }
    }
    depth_img.save(out_dir.as_ref().join("fallback_depth.png"))?;

    Ok(())
}