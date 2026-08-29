//! # Movement Subsystem Main Loop

mod calibration;
mod nav;
mod parsers;
mod types;
pub mod verification;

use std::f64::consts::PI;
use std::net::UdpSocket;
use std::time::Instant;

use calibration::SystemCalibrator;
use nav::NavigationManager;
use parsers::{parse_obstacle_json, parse_target_json};
use types::{ImuFrame, NavState, Point2D, RobotPose};
use verification::{
    verify_circuit_index_advance,
    verify_motor_buffer_offsets,
};

const CITY_CIRCUIT: &[Point2D] = &[
    Point2D { x: 0.0, y: 0.0 },
    Point2D { x: 8.0, y: 0.0 },
    Point2D { x: 0.0, y: 0.0 },
    Point2D { x: 0.0, y: 22.0 },
    Point2D { x: 22.0, y: 22.0 },
    Point2D { x: 22.0, y: -22.0 },
    Point2D { x: 0.0, y: -22.0 },
    Point2D { x: -5.0, y: 4.5 },
];

fn main() -> std::io::Result<()> {
    let imu_socket = UdpSocket::bind("127.0.0.1:5559")?;
    imu_socket.set_nonblocking(true)?;

    let obstacle_socket = UdpSocket::bind("127.0.0.1:5556")?;
    obstacle_socket.set_nonblocking(true)?;

    let target_socket = UdpSocket::bind("127.0.0.1:5560")?;
    target_socket.set_nonblocking(true)?;

    let send_socket = UdpSocket::bind("127.0.0.1:0")?;
    let sim_target = "127.0.0.1:5555";

    let mut robot = RobotPose::default();
    let mut calibrator = SystemCalibrator::new(50);
    let mut manager = NavigationManager::new();

    let mut imu_buf = [0u8; 52];
    let mut obstacle_buf = [0u8; 1024];
    let mut target_buf = [0u8; 256];

    let mut circuit_idx = 0;
    let mut last_time = Instant::now();

    let mut last_logged_state = NavState::Idle;
    let mut last_logged_pos = Point2D::default();
    let mut last_logged_heading = 0.0f64;

    println!("[Movement] System active.");
    println!(" -> Listening for IMU telemetry on UDP 127.0.0.1:5559");
    println!(" -> Listening for Vision Obstacles on UDP 127.0.0.1:5556");
    println!(" -> Listening for Click Targets on UDP 127.0.0.1:5560\n");

    loop {
        let dt = last_time.elapsed().as_secs_f64();
        last_time = Instant::now();

        while let Ok((amt, _)) = obstacle_socket.recv_from(&mut obstacle_buf) {
            if let Ok(payload_str) = std::str::from_utf8(&obstacle_buf[..amt]) {
                if let Some(obs_frame) = parse_obstacle_json(payload_str) {
                    if obs_frame.detected && obs_frame.distance_m > 0.0 {
                        manager.obstacle = obs_frame;
                    } else {
                        manager.obstacle.detected = false;
                    }
                }
            }
        }

        while let Ok((amt, _)) = target_socket.recv_from(&mut target_buf) {
            if let Ok(payload_str) = std::str::from_utf8(&target_buf[..amt]) {
                if let Some(custom_target) = parse_target_json(payload_str) {
                    println!(
                        "\n[CUSTOM TARGET OVERRIDE] Target updated to Coordinates: ({:.2}, {:.2})",
                        custom_target.x, custom_target.y
                    );
                    manager.set_target(custom_target);
                }
            }
        }

        while let Ok((amt, _)) = imu_socket.recv_from(&mut imu_buf) {
            if amt == 52 {
                let frame = ImuFrame::parse(&imu_buf);

                if !calibrator.is_calibrated {
                    let calibrated_now = calibrator.add_sample(frame, &mut robot);
                    if calibrated_now {
                        let initial_target = CITY_CIRCUIT[circuit_idx];
                        println!(
                            "[CIRCUIT START] Autonomous patrol active. Goal #0: ({:.2}, {:.2})",
                            initial_target.x, initial_target.y
                        );
                        manager.set_target(initial_target);
                    }
                } else {
                    robot.position.x = frame.pos_x as f64;
                    robot.position.y = frame.pos_y as f64;

                    let mut current_heading = (frame.yaw_rad as f64) * (180.0 / PI);
                    while current_heading > 180.0 { current_heading -= 360.0; }
                    while current_heading < -180.0 { current_heading += 360.0; }
                    robot.heading = current_heading;
                }
            }
        }

        if calibrator.is_calibrated {
            let command = manager.update(&robot, dt);

                if manager.state == NavState::Reached {
                circuit_idx = verify_circuit_index_advance(circuit_idx, CITY_CIRCUIT.len());
                let next_target = CITY_CIRCUIT[circuit_idx];
                println!(
                    "\n[CIRCUIT ADVANCE] Waypoint Reached! Advancing to Goal #{}: ({:.2}, {:.2})",
                    circuit_idx, next_target.x, next_target.y
                );
                manager.set_target(next_target);
            }

            debug_assert!(verification::verify_motor_buffer_offsets(0, 8));
            let mut buffer = [0u8; 16];
            buffer[0..8].copy_from_slice(&command.left_velocity.to_le_bytes());
            buffer[8..16].copy_from_slice(&command.right_velocity.to_le_bytes());
            let _ = send_socket.send_to(&buffer, sim_target);

            let pos_delta = (robot.position.x - last_logged_pos.x).hypot(robot.position.y - last_logged_pos.y);
            let heading_delta = (robot.heading - last_logged_heading).abs();

            if manager.state != last_logged_state || pos_delta > 0.05 || heading_delta > 0.5 {
                if let Some(target) = manager.target {
                    println!(
                        "[NAV] State: {:?} | Pos: ({:.2}, {:.2}) | Heading: {:.1}° | Target: ({:.2}, {:.2})",
                        manager.state, robot.position.x, robot.position.y, robot.heading, target.x, target.y
                    );
                }
                last_logged_state = manager.state;
                last_logged_pos = robot.position;
                last_logged_heading = robot.heading;
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}