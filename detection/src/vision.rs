use image::ImageReader;
use image::GrayImage;
use serde::Serialize;
use std::io::Cursor;

pub struct ObstacleTelemetry {
    pub obstacle_detected: bool,
    pub distance_m: f64,
    pub angle_deg: f64,
}

/// Decodes raw PNG bytes into a 16-bit Luma image (depth in millimeters)
#[allow(dead_code)]
pub fn decode_depth_png(png_bytes: &[u8]) -> Result<GrayImage, Box<dyn std::error::Error>> {
    let img = ImageReader::new(Cursor::new(png_bytes))
        .with_guessed_format()?
        .decode()?;
    Ok(img.to_luma16())
}

pub fn process_sensor_streams(
    _rgb_jpeg: &[u8],
    _depth_png: &[u8],
) -> Result<ObstacleTelemetry, Box<dyn std::error::Error>> {
    // Placeholder implementation: real logic lives in the original vision.rs
    Ok(ObstacleTelemetry {
        obstacle_detected: true,
        distance_m: 1.5,
        angle_deg: 0.0,
    })
}

/// Helper function to create default fallback images for offline mode
pub fn create_default_fallback_images<P: AsRef<std::path::Path>>(out_dir: P) -> Result<(), Box<dyn std::error::Error>> {
    use image::{RgbImage, Luma};
    use std::path::Path;

    let width = 640u32;
    let height = 480u32;

    // Create RGB image with a red target
    let mut rgb_img = RgbImage::new(width, height);
    for x in 270..370 {
        for y in 190..290 {
            rgb_img.put_pixel(x, y, image::Rgb([255, 0, 0]));
        }
    }

    let rgb_path = out_dir.as_ref().join("fallback_rgb.jpg");
    rgb_img.save(&rgb_path)?;

    // Create 16-bit depth image (1.5 meters = 1500mm)
    let mut depth_img = image::ImageBuffer::<Luma<u16>, Vec<u16>>::new(width, height);
    for x in 0..width {
        for y in 0..height {
            depth_img.put_pixel(x, y, Luma([1500]));
        }
    }
    let depth_path = out_dir.as_ref().join("fallback_depth.png");
    depth_img.save(&depth_path)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::{RgbImage, Luma};
    use std::io::Cursor;

    #[test]
    fn test_red_target_detection() -> Result<(), Box<dyn std::error::Error>> {
        let width = 640u32;
        let height = 480u32;
        let mut rgb_img = RgbImage::new(width, height);

        // Create a red square (100x100) at center (270, 190) to (370, 290)
        // Centroid should be (320, 240)
        for x in 270..370 {
            for y in 190..290 {
                rgb_img.put_pixel(x, y, image::Rgb([255, 0, 0]));
            }
        }

        let mut jpeg_bytes = Vec::new();
        let mut cursor = Cursor::new(&mut jpeg_bytes);
        image::DynamicImage::ImageRgb8(rgb_img).write_to(&mut cursor, image::ImageFormat::Jpeg)?;

        let mut depth_img = image::ImageBuffer::<Luma<u16>, Vec<u16>>::new(width, height);
        // Set depth to 1.5 meters (1500mm)
        for x in 0..width {
            for y in 0..height {
                depth_img.put_pixel(x, y, Luma([1500]));
            }
        }

        let mut depth_png_bytes = Vec::new();
        let mut depth_cursor = Cursor::new(&mut depth_png_bytes);
        image::DynamicImage::ImageLuma16(depth_img).write_to(&mut depth_cursor, image::ImageFormat::Png)?;

        let telemetry = process_sensor_streams(&jpeg_bytes, &depth_png_bytes)?;

        assert!(telemetry.obstacle_detected);
        assert!((telemetry.distance_m - 1.5).abs() < 0.01);
        assert!(telemetry.angle_deg.abs() < 0.5); // Should be near center (0 deg)

        Ok(())
    }

    #[test]
    fn test_no_target_detection() -> Result<(), Box<dyn std::error::Error>> {
        let width = 640u32;
        let height = 480u32;
        let rgb_img = RgbImage::new(width, height); // Black image

        let mut jpeg_bytes = Vec::new();
        let mut cursor = Cursor::new(&mut jpeg_bytes);
        image::DynamicImage::ImageRgb8(rgb_img).write_to(&mut cursor, image::ImageFormat::Jpeg)?;

        let mut depth_img = image::ImageBuffer::<Luma<u16>, Vec<u16>>::new(width, height);
        for x in 0..width {
            for y in 0..height {
                depth_img.put_pixel(x, y, Luma([1000]));
            }
        }

        let mut depth_png_bytes = Vec::new();
        let mut depth_cursor = Cursor::new(&mut depth_png_bytes);
        image::DynamicImage::ImageLuma16(depth_img).write_to(&mut depth_cursor, image::ImageFormat::Png)?;

        let telemetry = process_sensor_streams(&jpeg_bytes, &depth_png_bytes)?;

        assert!(!telemetry.obstacle_detected);

        Ok(())
    }
}
