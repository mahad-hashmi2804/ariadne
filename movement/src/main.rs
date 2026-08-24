use std::net::UdpSocket;
use std::thread::sleep;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let socket = UdpSocket::bind("127.0.0.1:0")?;
    let sim_target = "127.0.0.1:5555";

    println!("[Movement] Tracked controller online.");
    println!("[Movement] Streaming 16-byte [v_left, v_right] commands to UDP 5555...\n");

    let start_time = Instant::now();
    let track_width = 0.36; // Distance between left and right track centers in meters

    loop {
        let elapsed = start_time.elapsed().as_secs_f64();

        // Basic test routine: cycle through drive states every few seconds
        let (linear_v, angular_w) = match (elapsed as u64) % 12 {
            0..=3 => (2.0, 0.0),    // Drive straight forward (2.0 m/s)
            4..=7 => (1.5, 1.2),    // Arc turn right
            8..=11 => (0.0, 2.5),   // Pivot spin in place
            _ => (0.0, 0.0),
        };

        // 1. Explicitly type the outputs as f64
        let v_left: f64 = linear_v - (angular_w * track_width / 2.0);
        let v_right: f64 = linear_v + (angular_w * track_width / 2.0);

        let mut buffer = Vec::with_capacity(16);
        buffer.extend_from_slice(&v_left.to_le_bytes());
        buffer.extend_from_slice(&v_right.to_le_bytes());

        let _ = socket.send_to(&buffer, sim_target);
        sleep(Duration::from_millis(10)); // 100 Hz Loop
    }
}