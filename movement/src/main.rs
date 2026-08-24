mod gait;
mod ik;

use std::net::UdpSocket;
use std::thread::sleep;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let socket = UdpSocket::bind("127.0.0.1:0")?;
    let target_addr = "127.0.0.1:5555";

    println!("[Movement] Streaming straight-line forward gait...");

    let start_time = Instant::now();
    let freq = 1.2;

    loop {
        let sim_time = start_time.elapsed().as_secs_f64();
        let scale = (sim_time / 1.0).min(1.0);

        let t = (sim_time * freq) % 1.0;

        let (swing_a, lift_a, knee_a) = compute_gait(t, scale);
        let (swing_b, lift_b, knee_b) = compute_gait((t + 0.5) % 1.0, scale);

        let mut ctrl = [0.0f64; 18];

        // --- TRIPOD A ---
        // Left Legs (Positive Swing)
        ctrl[0]  =  swing_a; ctrl[1]  = lift_a; ctrl[2]  = knee_a; // FL
        ctrl[6]  =  swing_a; ctrl[7]  = lift_a; ctrl[8]  = knee_a; // BL

        // Right Leg (Inverted Swing for straight motion)
        ctrl[12] = -swing_a; ctrl[13] = lift_a; ctrl[14] = knee_a; // MR

        // --- TRIPOD B ---
        // Left Leg (Positive Swing)
        ctrl[3]  =  swing_b; ctrl[4]  = lift_b; ctrl[5]  = knee_b; // ML

        // Right Legs (Inverted Swing for straight motion)
        ctrl[9]  = -swing_b; ctrl[10] = lift_b; ctrl[11] = knee_b; // FR
        ctrl[15] = -swing_b; ctrl[16] = lift_b; ctrl[17] = knee_b; // BR

        // Pack 18 x f64 into 144 bytes
        let mut buffer = Vec::with_capacity(18 * 8);
        for val in ctrl {
            buffer.extend_from_slice(&val.to_le_bytes());
        }

        let _ = socket.send_to(&buffer, target_addr);
        sleep(Duration::from_millis(10));
    }
}

fn compute_gait(t: f64, scale: f64) -> (f64, f64, f64) {
    let max_stride = 30.0;
    if t < 0.5 {
        // STANCE PHASE: Foot on ground, pressing down (-0.15) & pushing backward
        let progress = t / 0.5;
        let swing = scale * (max_stride - progress * (2.0 * max_stride));
        let lift = scale * -2.65;
        let knee = scale * -0.05;
        (swing, lift, knee)
    } else {
        // SWING PHASE: Lift HIGH (+0.50) into air & bend knee (-0.60) to guarantee ground clearance
        let progress = (t - 0.5) / 0.5;
        let swing = scale * (-max_stride + progress * (2.0 * max_stride));
        let lift = scale * 30.0;
        let knee = scale * 4.0;
        (swing, lift, knee)
    }
}