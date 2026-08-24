mod vision;

use std::fs::File;
use std::io::Write;
use std::net::UdpSocket;
use std::path::Path;
use std::time::{Duration, Instant};
use vision::{create_default_fallback_images, process_sensor_streams};

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

    // Track stream health
    let mut last_frame_received = Instant::now();
    let fallback_timeout = Duration::from_secs(3);
    let mut fallback_active = false;

    loop {
        let loop_start = Instant::now();

        // Drain RGB Socket Buffer
        let mut got_rgb = false;
        if let Ok((amt, _)) = rgb_socket.recv_from(&mut rgb_buf) {
            latest_rgb = Some(rgb_buf[..amt].to_vec());
            got_rgb = true;
        }

        // Drain Depth Socket Buffer
        let mut got_depth = false;
        if let Ok((amt, _)) = depth_socket.recv_from(&mut depth_buf) {
            latest_depth = Some(depth_buf[..amt].to_vec());
            got_depth = true;
        }

        if got_rgb || got_depth {
            last_frame_received = Instant::now();
            if fallback_active {
                println!("[Detection] Live UDP stream resumed. Disengaging offline fallback.");
                fallback_active = false;
            }
        }

        // Engage fallback if stream timeout is exceeded
        if last_frame_received.elapsed() >= fallback_timeout {
            if !fallback_active {
                println!("[Detection] Live UDP stream dropped. Engaging offline fallback...");
                fallback_active = true;
            }

            // Ensure static fallback files exist
            if !Path::new("fallback_rgb.jpg").exists() || !Path::new("fallback_depth.png").exists() {
                println!("[Detection] Static fallback files missing. Generating default targets...");
                create_default_fallback_images()?;
            }

            // Load static fallback frames
            if let (Ok(rgb_data), Ok(depth_data)) = (std::fs::read("fallback_rgb.jpg"), std::fs::read("fallback_depth.png")) {
                latest_rgb = Some(rgb_data);
                latest_depth = Some(depth_data);
            }
        }

        // Process whenever both active frame buffers are available
        if let (Some(rgb_data), Some(depth_data)) = (&latest_rgb, &latest_depth) {
            if let Ok(telemetry) = process_sensor_streams(rgb_data, depth_data) {
                // Broadcast JSON payload to Movement on Port 5556
                let json_payload = serde_json::to_string(&telemetry)?;
                let _ = telemetry_socket.send_to(json_payload.as_bytes(), movement_target);

                // Save 5 paired test frame files spaced 5 seconds apart (only during live UDP streams)
                if !fallback_active {
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
            }

            // Flush local frame buffers (if in fallback, we keep loading them each cycle)
            if !fallback_active {
                latest_rgb = None;
                latest_depth = None;
            }
        }

        // Enforce 30Hz (33.33ms per cycle)
        let elapsed = loop_start.elapsed();
        let target_duration = Duration::from_micros(33333);
        if elapsed < target_duration {
            std::thread::sleep(target_duration - elapsed);
        }
    }
}