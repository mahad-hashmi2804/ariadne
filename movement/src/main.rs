use std::f64::consts::PI;
use std::net::UdpSocket;
use std::time::Instant;

const CITY_CIRCUIT: &[Point2D] = &[
    Point2D { x: 0.0, y: 0.0 },     // Waypoint 0: Central Plaza
    Point2D { x: 0.0, y: 22.0 },    // Waypoint 1: North Avenue
    Point2D { x: 22.0, y: 22.0 },   // Waypoint 2: East Street
    Point2D { x: 22.0, y: -22.0 },  // Waypoint 3: South-East Sector
    Point2D { x: 0.0, y: -22.0 },   // Waypoint 4: South Avenue
    Point2D { x: -5.0, y: 4.5 },    // Waypoint 5: North-West Rubble Alley
];

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct RobotPose {
    pub position: Point2D,
    pub heading: f64,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ImuFrame {
    pub timestamp: f32,
    pub accel: [f32; 3],
    pub gyro: [f32; 3],
    pub mag: [f32; 3],
    pub pos_x: f32,
    pub pos_y: f32,
    pub yaw_rad: f32,
}

impl ImuFrame {
    pub fn parse(buf: &[u8; 52]) -> Self {
        let mut floats = [0.0f32; 13];
        for i in 0..13 {
            let chunk = &buf[i * 4..(i + 1) * 4];
            floats[i] = f32::from_le_bytes(chunk.try_into().unwrap());
        }

        Self {
            timestamp: floats[0],
            accel: [floats[1], floats[2], floats[3]],
            gyro: [floats[4], floats[5], floats[6]],
            mag: [floats[7], floats[8], floats[9]],
            pos_x: floats[10],
            pos_y: floats[11],
            yaw_rad: floats[12],
        }
    }
}

pub struct SystemCalibrator {
    samples_required: usize,
    collected_samples: Vec<ImuFrame>,
    pub gyro_bias_z: f64,
    pub is_calibrated: bool,
}

impl SystemCalibrator {
    pub fn new(samples: usize) -> Self {
        Self {
            samples_required: samples,
            collected_samples: Vec::with_capacity(samples),
            gyro_bias_z: 0.0,
            is_calibrated: false,
        }
    }

    pub fn add_sample(&mut self, frame: ImuFrame, robot: &mut RobotPose) -> bool {
        if self.is_calibrated {
            return true;
        }

        self.collected_samples.push(frame);
        print!(
            "\r[CALIBRATING] Sampling IMU & Pose... ({}/{})",
            self.collected_samples.len(),
            self.samples_required
        );

        if self.collected_samples.len() >= self.samples_required {
            let sum_gz: f64 = self.collected_samples.iter().map(|f| f.gyro[2] as f64).sum();
            self.gyro_bias_z = sum_gz / (self.collected_samples.len() as f64);

            let last_frame = self.collected_samples.last().unwrap();
            robot.position.x = last_frame.pos_x as f64;
            robot.position.y = last_frame.pos_y as f64;

            let mut init_heading = (last_frame.yaw_rad as f64) * (180.0 / PI);
            while init_heading > 180.0 { init_heading -= 360.0; }
            while init_heading < -180.0 { init_heading += 360.0; }
            robot.heading = init_heading;

            self.is_calibrated = true;
            println!("\n[CALIBRATION COMPLETE]");
            println!(" -> Gyro Z Bias: {:.6} rad/s", self.gyro_bias_z);
            println!(
                " -> Spawn Pose Synced: ({:.2}, {:.2}) @ {:.1}°\n",
                robot.position.x, robot.position.y, robot.heading
            );
        }

        self.is_calibrated
    }
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum NavState {
    Idle,
    Turning,
    Moving,
    Reached,
}

pub struct NavCommand {
    pub left_velocity: f64,
    pub right_velocity: f64,
}

pub struct NavigationManager {
    pub state: NavState,
    pub target: Option<Point2D>,
    pub base_speed: f64,
    pub max_turn_speed: f64,
    pub min_turn_speed: f64,
    pub decel_angle_deg: f64,
    pub angle_tolerance_deg: f64,
    pub distance_tolerance_m: f64,

    // Velocity acceleration limits
    pub current_left_v: f64,
    pub current_right_v: f64,
    pub max_accel: f64,
}

impl NavigationManager {
    pub fn new() -> Self {
        Self {
            state: NavState::Idle,
            target: None,
            base_speed: 500.0,             // Cruise velocity (m/s scale)
            max_turn_speed: 1.4,         // Fast turn speed for large angles (>45 deg)
            min_turn_speed: 0.25,        // Creep speed to overcome friction near target
            decel_angle_deg: 45.0,       // Angle at which turning begins to decelerate
            angle_tolerance_deg: 2.5,    // Lock-in heading tolerance window
            distance_tolerance_m: 0.30,  // Target arrival radius
            current_left_v: 0.0,
            current_right_v: 0.0,
            max_accel: 100.0,             // Acceleration rate limit
        }
    }

    pub fn set_target(&mut self, target: Point2D) {
        self.target = Some(target);
        self.state = NavState::Turning;
    }

    pub fn update(&mut self, robot: &RobotPose, dt: f64) -> NavCommand {
        let target = match self.target {
            Some(t) => t,
            None => {
                self.state = NavState::Idle;
                return self.ramp_velocities(0.0, 0.0, dt);
            }
        };

        let dx = target.x - robot.position.x;
        let dy = target.y - robot.position.y;
        let dist = (dx * dx + dy * dy).sqrt();

        if dist < self.distance_tolerance_m {
            self.state = NavState::Reached;
            self.target = None;
            return self.ramp_velocities(0.0, 0.0, dt);
        }

        let target_angle_rad = dy.atan2(dx);
        let target_angle_deg = target_angle_rad * (180.0 / PI);

        let mut angle_diff = target_angle_deg - robot.heading;
        while angle_diff > 180.0 { angle_diff -= 360.0; }
        while angle_diff < -180.0 { angle_diff += 360.0; }

        let (target_left, target_right) = match self.state {
            NavState::Turning => {
                if angle_diff.abs() <= self.angle_tolerance_deg {
                    self.state = NavState::Moving;
                    (self.base_speed, self.base_speed)
                } else {
                    // Proportional Deceleration Curve:
                    // Scale from max_turn_speed down to min_turn_speed over decel_angle_deg
                    let scale = (angle_diff.abs() / self.decel_angle_deg).clamp(0.0, 1.0);
                    let dynamic_turn_speed = self.min_turn_speed + (self.max_turn_speed - self.min_turn_speed) * scale;

                    if angle_diff > 0.0 {
                        (-dynamic_turn_speed, dynamic_turn_speed)
                    } else {
                        (dynamic_turn_speed, -dynamic_turn_speed)
                    }
                }
            }
            NavState::Moving => {
                if angle_diff.abs() > self.angle_tolerance_deg * 3.0 {
                    self.state = NavState::Turning;
                    (0.0, 0.0)
                } else {
                    let k_p = 0.02;
                    let steering = angle_diff * k_p;
                    (
                        (self.base_speed - steering).clamp(-2.5, 2.5),
                        (self.base_speed + steering).clamp(-2.5, 2.5),
                    )
                }
            }
            _ => (0.0, 0.0),
        };

        self.ramp_velocities(target_left, target_right, dt)
    }

    fn ramp_velocities(&mut self, target_left: f64, target_right: f64, dt: f64) -> NavCommand {
        let max_step = self.max_accel * dt;

        let d_left = target_left - self.current_left_v;
        self.current_left_v += d_left.clamp(-max_step, max_step);

        let d_right = target_right - self.current_right_v;
        self.current_right_v += d_right.clamp(-max_step, max_step);

        NavCommand {
            left_velocity: self.current_left_v,
            right_velocity: self.current_right_v,
        }
    }
}

fn parse_target_json(payload: &str) -> Option<Point2D> {
    let clean = payload.trim_matches(|c| c == '{' || c == '}' || c == ' ' || c == '\n' || c == '\r');
    let mut x = None;
    let mut y = None;

    for kv in clean.split(',') {
        let parts: Vec<&str> = kv.split(':').collect();
        if parts.len() == 2 {
            let key = parts[0].trim().trim_matches('"');
            let val: f64 = parts[1].trim().parse().ok()?;
            if key == "x" { x = Some(val); }
            if key == "y" { y = Some(val); }
        }
    }

    if let (Some(x), Some(y)) = (x, y) {
        Some(Point2D { x, y })
    } else {
        None
    }
}

fn main() -> std::io::Result<()> {
    let imu_socket = UdpSocket::bind("127.0.0.1:5559")?;
    imu_socket.set_nonblocking(true)?;

    let target_socket = UdpSocket::bind("127.0.0.1:5560")?;
    target_socket.set_nonblocking(true)?;

    let send_socket = UdpSocket::bind("127.0.0.1:0")?;
    let sim_target = "127.0.0.1:5555";

    let mut robot = RobotPose::default();
    let mut calibrator = SystemCalibrator::new(50);
    let mut manager = NavigationManager::new();

    let mut imu_buf = [0u8; 52];
    let mut target_buf = [0u8; 256];

    let mut circuit_idx = 0;
    let mut last_time = Instant::now();

    let mut last_logged_state = NavState::Idle;
    let mut last_logged_pos = Point2D::default();
    let mut last_logged_heading = 0.0f64;

    println!("[Movement] System active. Listening on UDP 5559 (IMU) & UDP 5560 (Targets)...");

    loop {
        let dt = last_time.elapsed().as_secs_f64();
        last_time = Instant::now();

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
                circuit_idx = (circuit_idx + 1) % CITY_CIRCUIT.len();
                let next_target = CITY_CIRCUIT[circuit_idx];
                println!(
                    "\n[CIRCUIT ADVANCE] Waypoint Reached! Advancing to Goal #{}: ({:.2}, {:.2})",
                    circuit_idx, next_target.x, next_target.y
                );
                manager.set_target(next_target);
            }

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