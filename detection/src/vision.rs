use image::load_from_memory;
use serde::Serialize;

#[derive(Serialize, Debug)]
pub struct ObstacleTelemetry {
    pub obstacle_detected: bool,
    pub distance_m: f64,
    pub angle_deg: f64,
}

/// Analyzes incoming depth PNG & RGB JPEG buffers to detect obstacles in front of the robot.
pub fn process_sensor_streams(
    _jpeg_bytes: &[u8],
    depth_png_bytes: &[u8],
) -> Result<ObstacleTelemetry, Box<dyn std::error::Error>> {
    // 1. Decode 16-bit single-channel PNG depth buffer (Values are in millimeters)
    let depth_img = load_from_memory(depth_png_bytes)?.to_luma16();
    let (width, height) = depth_img.dimensions();

    let center_x = (width / 2) as f64;
    let mut min_distance_mm = u16::MAX;
    let mut closest_pixel_x: u32 = width / 2;
    let mut obstacle_pixel_count = 0;

    // 2. Define Region of Interest (ROI): Lock horizon window to exclude floor plane
    let min_x = (width as f64 * 0.25) as u32;
    let max_x = (width as f64 * 0.75) as u32;
    let min_y = (height as f64 * 0.25) as u32; // Skip top sky/ceiling noise
    let max_y = (height as f64 * 0.52) as u32; // Cut off at horizon (strips floor plane at 0.49m)

    // Thresholds: Ignore everything under 600mm (0.60m) to purge ground plane returns
    let min_valid_mm = 600u16;
    let max_valid_mm = 2500u16;

    for y in min_y..max_y {
        for x in min_x..max_x {
            let depth_mm = depth_img.get_pixel(x, y)[0];

            // Filter ground plane reflections and far noise
            if depth_mm >= min_valid_mm && depth_mm < max_valid_mm {
                obstacle_pixel_count += 1;

                if depth_mm < min_distance_mm {
                    min_distance_mm = depth_mm;
                    closest_pixel_x = x;
                }
            }
        }
    }

    // 3. Evaluate detection result (require at least 50 positive depth pixels to filter sensor noise)
    if obstacle_pixel_count > 50 {
        let distance_m = (min_distance_mm as f64) / 1000.0;

        // Map pixel X coordinate to relative horizontal bearing (-30° to +30°)
        let angle_deg = (((closest_pixel_x as f64) - center_x) / center_x) * 30.0;

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