use serde::Deserialize;
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

#[derive(Debug, Deserialize)]
struct DetectionPacket {
    #[serde(rename = "obstacle_detected")]
    survivor_detected: bool,
    distance_m: f64,
    angle_deg: f64,
}

pub fn run_telemetry_listener(override_flag: Arc<AtomicBool>) -> std::io::Result<()> {
    let socket = UdpSocket::bind("127.0.0.1:5556")?;
    let mut buf = [0u8; 1024];

    println!("Telemetry listener running on {}", socket.local_addr()?);

    loop {
        let (num_bytes, src_addr) = socket.recv_from(&mut buf)?;
        let data = &buf[..num_bytes];

        match serde_json::from_slice::<DetectionPacket>(data) {
            Ok(packet) => {
                println!("Received from {}: {:?}", src_addr, packet);

                // Engaged if an obstacle is detected and it is within a dangerous range (e.g., < 2.0m)
                if packet.survivor_detected && packet.distance_m < 2.0 {
                    override_flag.store(true, Ordering::SeqCst);
                    println!("Survivor/Obstacle detected at {:.2}m => override engaged", packet.distance_m);
                }
            }
            Err(e) => {
                eprintln!("Failed to parse packet: {} (raw bytes: {:?})", e, data);
            }
        }
    }
}

