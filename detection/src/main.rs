mod vision;

use std::fs::File;
use std::io::Write;
use std::net::UdpSocket;
use std::time::{Duration, Instant};
use vision::{process_sensor_streams, ObstacleTelemetry};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Bind Dual Receiver Sockets
    let rgb_socket = UdpSocket::bind("127.0.0.1:5557")?;
    let depth_socket = UdpSocket::bind("127.0.0.1:5558")?;

    // Set non-blocking mode to easily pair frame arrivals
    rgb_socket.set_nonblocking(true)?;
    depth_socket.set_nonblocking(true)?;

    // Telemetry Broadcast Socket to Movement (Port 5556)
    let telemetry_socket = UdpSocket::bind("127.0.0.1:0")?;
    let movement_target = "127.0.0.1:5556";

    println!("[Detection] Engine online.");
    println!(" -> Listening for RGB frames on UDP 127.0.0.1:5557");
    println!(" -> Listening for Depth frames on UDP 127.0.0.1:5558");
    println!(" -> Broadcasting telemetry to UDP 127.0.0.1:5556\n");

    let mut rgb_buf = [0u8; 65535];
    let mut depth_buf = [0u8; 65535];

    let mut saved_count = 0;
    let total_test_captures = 5;
    let capture_interval = Duration::from_secs(5);
    let mut last_saved_time = Instant::now() - capture_interval;

    let mut latest_rgb: Option<Vec<u8>> = None;
    let mut latest_depth: Option<Vec<u8>> = None;

    loop {
        // Drain RGB Socket Buffer
        if let Ok((amt, _)) = rgb_socket.recv_from(&mut rgb_buf) {
            latest_rgb = Some(rgb_buf[..amt].to_vec());
        }

        // Drain Depth Socket Buffer
        if let Ok((amt, _)) = depth_socket.recv_from(&mut depth_buf) {
            latest_depth = Some(depth_buf[..amt].to_vec());
        }

        // Process whenever both active frame buffers are available
        if let (Some(rgb_data), Some(depth_data)) = (&latest_rgb, &latest_depth) {
            if let Ok(telemetry) = process_sensor_streams(rgb_data, depth_data) {
                // Broadcast JSON payload to Movement on Port 5556
                let json_payload = serde_json::to_string(&telemetry)?;
                let _ = telemetry_socket.send_to(json_payload.as_bytes(), movement_target);

                // Save 5 paired test frame files spaced 5 seconds apart
                let now = Instant::now();
                if saved_count < total_test_captures && now.duration_since(last_saved_time) >= capture_interval {
                    saved_count += 1;
                    last_saved_time = now;

                    let rgb_filename = format!("frame_{}_rgb.jpg", saved_count);
                    let depth_filename = format!("frame_{}_depth.png", saved_count);

                    let mut rgb_file = File::create(&rgb_filename)?;
                    rgb_file.write_all(rgb_data)?;

                    let mut depth_file = File::create(&depth_filename)?;
                    depth_file.write_all(depth_data)?;

                    println!(
                        "[{}/{}] Saved paired frames: {} & {} | Distance: {:.2}m",
                        saved_count, total_test_captures, rgb_filename, depth_filename, telemetry.distance_m
                    );
                }
            }

            // Flush local frame buffers
            latest_rgb = None;
            latest_depth = None;
        }

        std::thread::sleep(Duration::from_millis(5));
    }
}