//! # Vision Processing & Depth Extraction Module
//!
//! Provides PNG depth decoding, region-of-interest horizon filtering, centroid offset calculation,
//! and fallback asset generation for offline sensor streams.

use image::{ImageBuffer, ImageReader, Luma, Rgb, RgbImage};
use serde::Serialize;
use std::io::Cursor;

use crate::verification::{compute_angle_milli_deg, compute_centroid, compute_roi_bounds, depth_in_range, PixelAccumulator};


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

pub fn process_sensor_streams(
    _jpeg_bytes: &[u8],
    depth_png_bytes: &[u8],
) -> Result<ObstacleTelemetry, Box<dyn std::error::Error>> {
    let depth_image = decode_depth_png(depth_png_bytes)?;
    let (width, height) = depth_image.dimensions();

    let roi = compute_roi_bounds(width, height);

    let min_dist_mm = 300u16;  // Minimum detection threshold (0.3m)
    let max_dist_mm = 2000u16; // Maximum detection horizon (2.0m)

    let mut acc = PixelAccumulator::new();

    for y in roi.y_start..roi.y_end {
        for x in roi.x_start..roi.x_end {
            let depth_mm = depth_image.get_pixel(x, y)[0];
            if depth_in_range(depth_mm, min_dist_mm, max_dist_mm) {
                acc.accumulate(depth_mm, x);
            }
        }
    }

    if acc.close_pixel_count > 100 {
        let (avg_depth_mm, centroid_x) = compute_centroid(&acc);
        let angle_milli_deg = compute_angle_milli_deg(centroid_x as u32, width);

        let distance_m = avg_depth_mm as f64 / 1000.0;
        let angle_deg = angle_milli_deg as f64 / 1000.0;

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