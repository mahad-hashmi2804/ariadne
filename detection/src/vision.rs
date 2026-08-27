use image::ImageReader;
use image::Luma;
use serde::Serialize;
use std::io::Cursor;

#[derive(Serialize)]
pub struct ObstacleTelemetry {
    pub obstacle_detected: bool,
    pub distance_m: f64,
    pub angle_deg: f64,
}

pub fn decode_depth_png(png_bytes: &[u8]) -> Result<image::ImageBuffer<Luma<u16>, Vec<u16>>, Box<dyn std::error::Error>> {
    let img = ImageReader::new(Cursor::new(png_bytes))
        .with_guessed_format()?
        .decode()?;
    Ok(img.to_luma16())
}

pub fn process_sensor_streams(
    _jpeg_bytes: &[u8],
    depth_png_bytes: &[u8],
) -> Result<ObstacleTelemetry, Box<dyn std::error::Error>> {
    let depth_img = ImageReader::new(Cursor::new(depth_png_bytes))
        .with_guessed_format()?
        .decode()?
        .to_luma16();

    let (width, height) = depth_img.dimensions();

    // Restrict vertical scan to upper/middle horizon band (12% to 42% height)
    // This completely ignores the ground plane below camera height (z = 0.085m)
    let x_start = width * 20 / 100;
    let x_end = width * 80 / 100;
    let y_start = height * 12 / 100;
    let y_end = height * 42 / 100;

    let mut close_pixel_count: u64 = 0;
    let mut sum_depth_mm: u64 = 0;
    let mut sum_x: u64 = 0;

    let min_dist_mm = 300u16;   // 0.3m minimum distance threshold
    let max_dist_mm = 2000u16;  // 2.0m maximum detection horizon

    for y in y_start..y_end {
        for x in x_start..x_end {
            let depth_mm = depth_img.get_pixel(x, y)[0];
            if depth_mm >= min_dist_mm && depth_mm <= max_dist_mm {
                close_pixel_count += 1;
                sum_depth_mm += depth_mm as u64;
                sum_x += x as u64;
            }
        }
    }

    if close_pixel_count > 100 {
        let avg_depth_mm = (sum_depth_mm / close_pixel_count) as f64;
        let centroid_x = (sum_x / close_pixel_count) as f64;
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

pub fn create_default_fallback_images<P: AsRef<std::path::Path>>(out_dir: P) -> Result<(), Box<dyn std::error::Error>> {
    use image::{RgbImage, Luma};

    let width = 640u32;
    let height = 480u32;

    let mut rgb_img = RgbImage::new(width, height);
    for x in 270..370 {
        for y in 190..290 {
            rgb_img.put_pixel(x, y, image::Rgb([255, 0, 0]));
        }
    }
    rgb_img.save(out_dir.as_ref().join("fallback_rgb.jpg"))?;

    let mut depth_img = image::ImageBuffer::<Luma<u16>, Vec<u16>>::new(width, height);
    for x in 0..width {
        for y in 0..height {
            depth_img.put_pixel(x, y, Luma([1500]));
        }
    }
    depth_img.save(out_dir.as_ref().join("fallback_depth.png"))?;

    Ok(())
}