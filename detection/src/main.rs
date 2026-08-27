mod vision;

use std::net::UdpSocket;
use std::thread::sleep;
use std::time::Duration;
use vision::{process_sensor_streams, ObstacleTelemetry};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Dual receiver sockets for vision streams from Simulation
    let rgb_socket = UdpSocket::bind("127.0.0.1:5557")?;
    let depth_socket = UdpSocket::bind("127.0.0.1:5558")?;

    rgb_socket.set_nonblocking(true)?;
    depth_socket.set_nonblocking(true)?;

    // Outbound telemetry socket to Movement
    let telemetry_socket = UdpSocket::bind("127.0.0.1:0")?;
    let movement_target = "127.0.0.1:5556";

    println!("========================================");
    println!("       ARIADNE VISION ENGINE            ");
    println!("========================================");
    println!("[Detection] Online and operational.");
    println!(" -> Receiving RGB feed on UDP 127.0.0.1:5557");
    println!(" -> Receiving Depth feed on UDP 127.0.0.1:5558");
    println!(" -> Streaming telemetry JSON to UDP 127.0.0.1:5556\n");

    let mut rgb_buf = [0u8; 65535];
    let mut depth_buf = [0u8; 65535];

    let mut latest_rgb: Option<Vec<u8>> = None;
    let mut latest_depth: Option<Vec<u8>> = None;

    loop {
        // Drain latest RGB frame packet
        if let Ok((amt, _)) = rgb_socket.recv_from(&mut rgb_buf) {
            latest_rgb = Some(rgb_buf[..amt].to_vec());
        }

        // Drain latest Depth frame packet
        if let Ok((amt, _)) = depth_socket.recv_from(&mut depth_buf) {
            latest_depth = Some(depth_buf[..amt].to_vec());
        }

        // Process frames as soon as both buffers are available
        if let (Some(rgb_data), Some(depth_data)) = (&latest_rgb, &latest_depth) {
            if let Ok(telemetry) = process_sensor_streams(rgb_data, depth_data) {
                // Serialize and broadcast telemetry payload over UDP
                let json_payload = serde_json::to_string(&telemetry)?;
                let _ = telemetry_socket.send_to(json_payload.as_bytes(), movement_target);

                if telemetry.obstacle_detected {
                    println!(
                        "[VISION ALERT] Obstacle at {:.2}m | Bearing: {:.1}°",
                        telemetry.distance_m, telemetry.angle_deg
                    );
                }
            }

            // Flush local buffers for next loop
            latest_rgb = None;
            latest_depth = None;
        }

        sleep(Duration::from_millis(10)); // ~100 Hz polling loop
    }
}