use image::io::Reader as ImageReader;
use image::GrayImage;
use serde::Serialize;
use std::io::Cursor;

#[derive(Serialize, Debug)]
pub struct ObstacleTelemetry {
    pub obstacle_detected: bool,
    pub distance_m: f64,
    pub angle_deg: f64,
}

/// Decodes raw PNG bytes into a 16-bit Luma image (depth in millimeters)
pub fn decode_depth_png(png_bytes: &[u8]) -> Result<GrayImage, Box<dyn std::error::Error>> {
    let img = ImageReader::new(Cursor::new(png_bytes))
        .with_guessed_format()?
        .decode()?;

    // Returns 8-bit image representation for debug/visualization saving
    Ok(img.to_luma8())
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