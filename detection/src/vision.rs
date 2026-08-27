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

/// Decodes raw PNG bytes into a 16-bit Luma image (depth in millimeters)
#[allow(dead_code)]
pub fn decode_depth_png(png_bytes: &[u8]) -> Result<image::ImageBuffer<Luma<u16>, Vec<u16>>, Box<dyn std::error::Error>> {
    let img = ImageReader::new(Cursor::new(png_bytes))
        .with_guessed_format()?
        .decode()?;
    Ok(img.to_luma16())
}

/// Analyzes JPEG color buffer and sample depth map at the object centroid
pub fn process_sensor_streams(
    jpeg_bytes: &[u8],
    depth_png_bytes: &[u8],
) -> Result<ObstacleTelemetry, Box<dyn std::error::Error>> {
    // 1. Decode RGB Image
    let rgb_img = ImageReader::new(Cursor::new(jpeg_bytes))
        .with_guessed_format()?
        .decode()?
        .to_rgb8();

    let (width, height) = rgb_img.dimensions();

    // 2. Decode 16-Bit Depth Image (Values are millimeters)
    let depth_img = ImageReader::new(Cursor::new(depth_png_bytes))
        .with_guessed_format()?
        .decode()?
        .to_luma16();

    let mut red_pixel_count = 0;
    let mut sum_x: u64 = 0;
    let mut sum_y: u64 = 0;

    // 3. Segment Red Pixels
    for (x, y, pixel) in rgb_img.enumerate_pixels() {
        let r = pixel[0] as f32;
        let g = pixel[1] as f32;
        let b = pixel[2] as f32;

        if r > 140.0 && g < 80.0 && b < 80.0 {
            red_pixel_count += 1;
            sum_x += x as u64;
            sum_y += y as u64;
        }
    }

    // 4. Calculate Spatial Metrics
    if red_pixel_count > 250 {
        let centroid_x = (sum_x / red_pixel_count) as u32;
        let centroid_y = (sum_y / red_pixel_count) as u32;

        // Clamp centroid pixel coordinates safely within frame dimensions
        let safe_x = centroid_x.min(width - 1);
        let safe_y = centroid_y.min(height - 1);

        // Sample true metric distance from depth buffer at object centroid
        let depth_mm = depth_img.get_pixel(safe_x, safe_y)[0] as f64;
        let distance_m = depth_mm / 1000.0;

        // Compute horizontal bearing angle relative to center axis (-30° to +30°)
        let center_x = (width / 2) as f64;
        let angle_deg = (((safe_x as f64) - center_x) / center_x) * 30.0;

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

/// Helper function to create default fallback images for offline mode
pub fn create_default_fallback_images<P: AsRef<std::path::Path>>(out_dir: P) -> Result<(), Box<dyn std::error::Error>> {
    use image::{RgbImage, Luma};

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
