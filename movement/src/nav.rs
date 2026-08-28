use std::f64::consts::PI;
use std::time::Duration;
use crate::types::{NavCommand, NavState, ObstacleFrame, Point2D, RobotPose};

// -----------------------------------------------------------------------------
// VERIFIED ALGORITHM IMPLEMENTATIONS (Mirrors movement/src/verification.rs)
// -----------------------------------------------------------------------------
fn verify_angle_normalize(angle: i32) -> i32 {
    let mut a = angle;
    if a > 180 { a -= 360; }
    if a < -180 { a += 360; }
    a
}

fn verify_accel_ramp(current_v: i32, target_v: i32, max_step: i32) -> i32 {
    let diff = target_v - current_v;
    let clamped_step = if diff > max_step {
        max_step
    } else if diff < -max_step {
        -max_step
    } else {
        diff
    };
    current_v + clamped_step
}

fn verify_bypass_distance(obstacle_depth_mm: i32, safety_buffer_mm: i32) -> i32 {
    obstacle_depth_mm + safety_buffer_mm
}

fn verify_depth_threshold(depth_mm: i32, min_mm: i32, max_mm: i32) -> bool {
    depth_mm >= min_mm && depth_mm <= max_mm
}

fn verify_differential_steering(_base_v: i32, steering: i32, max_split: i32) -> i32 {
    if steering > max_split {
        max_split
    } else if steering < -max_split {
        -max_split
    } else {
        steering
    }
}

// -----------------------------------------------------------------------------
// NAVIGATION ENGINE
// -----------------------------------------------------------------------------
pub struct NavigationManager {
    pub state: NavState,
    pub target: Option<Point2D>,
    pub base_speed: f64,
    pub max_turn_speed: f64,
    pub min_turn_speed: f64,
    pub decel_angle_deg: f64,
    pub angle_tolerance_deg: f64,
    pub distance_tolerance_m: f64,

    pub obstacle: ObstacleFrame,
    pub critical_obstacle_dist_m: f64,
    pub avoid_turn_dir: f64,
    pub avoid_start_heading: f64,
    pub max_avoid_turn_deg: f64,
    pub last_obstacle_dist: f64,
    pub bypass_start_pos: Point2D,
    pub bypass_target_dist: f64,

    pub current_left_v: f64,
    pub current_right_v: f64,
    pub max_accel: f64,
}

impl NavigationManager {
    pub fn new() -> Self {
        Self {
            state: NavState::Idle,
            target: None,
            base_speed: 1.8,
            max_turn_speed: 1.2,
            min_turn_speed: 0.25,
            decel_angle_deg: 45.0,
            angle_tolerance_deg: 2.5,
            distance_tolerance_m: 0.30,

            obstacle: ObstacleFrame::default(),
            critical_obstacle_dist_m: 1.5,
            avoid_turn_dir: 1.0,
            avoid_start_heading: 0.0,
            max_avoid_turn_deg: 45.0,
            last_obstacle_dist: 1.2,
            bypass_start_pos: Point2D::default(),
            bypass_target_dist: 0.0,

            current_left_v: 0.0,
            current_right_v: 0.0,
            max_accel: 15.0,
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

        // Verified depth thresholding
        let obs_mm = (self.obstacle.distance_m * 1000.0) as i32;
        let valid_depth = verify_depth_threshold(obs_mm, 300, 2500);

        let has_active_obstacle = self.obstacle.detected
            && valid_depth
            && self.obstacle.last_seen.map_or(false, |t| t.elapsed() < Duration::from_millis(400));

        match self.state {
            NavState::AvoidingTurn => {
                if has_active_obstacle {
                    self.last_obstacle_dist = self.obstacle.distance_m;
                }

                let mut turned_deg = (robot.heading - self.avoid_start_heading).abs();
                if turned_deg > 180.0 { turned_deg = 360.0 - turned_deg; }

                if !has_active_obstacle || turned_deg >= self.max_avoid_turn_deg {
                    self.bypass_start_pos = robot.position;

                    // Verified bypass calculation
                    let obstacle_mm = (self.last_obstacle_dist * 1000.0).clamp(100.0, 10000.0) as i32;
                    let verified_clearance_mm = verify_bypass_distance(obstacle_mm, 500);
                    self.bypass_target_dist = (verified_clearance_mm as f64) / 1000.0;

                    self.state = NavState::AvoidingBypass;

                    println!(
                        "\n[AVOIDANCE] Pivot complete ({:.1}° turned). Driving {:.2}m straight to bypass barrier...",
                        turned_deg, self.bypass_target_dist
                    );
                } else {
                    let turn_cmd = self.max_turn_speed * self.avoid_turn_dir;
                    return self.ramp_velocities(-turn_cmd, turn_cmd, dt);
                }
            }

            NavState::AvoidingBypass => {
                let driven = (robot.position.x - self.bypass_start_pos.x)
                    .hypot(robot.position.y - self.bypass_start_pos.y);

                if driven >= self.bypass_target_dist {
                    println!(
                        "\n[AVOIDANCE COMPLETE] Cleared {:.2}m. Recalculating path to target from ({:.2}, {:.2})...",
                        driven, robot.position.x, robot.position.y
                    );
                    self.state = NavState::Turning;
                } else {
                    let speed = self.base_speed * 0.75;
                    return self.ramp_velocities(speed, speed, dt);
                }
            }

            _ => {
                if has_active_obstacle && self.obstacle.distance_m < self.critical_obstacle_dist_m {
                    self.avoid_turn_dir = if self.obstacle.angle_deg >= 0.0 { -1.0 } else { 1.0 };
                    self.avoid_start_heading = robot.heading;
                    self.last_obstacle_dist = self.obstacle.distance_m;
                    self.state = NavState::AvoidingTurn;

                    println!(
                        "\n[AVOIDANCE TRIGGERED] Obstacle at {:.2}m (Angle: {:.1}°). Pivoting away...",
                        self.obstacle.distance_m, self.obstacle.angle_deg
                    );

                    let turn_cmd = self.max_turn_speed * self.avoid_turn_dir;
                    return self.ramp_velocities(-turn_cmd, turn_cmd, dt);
                }
            }
        }

        let target_angle_rad = dy.atan2(dx);
        let target_angle_deg = target_angle_rad * (180.0 / PI);

        // Verified angle normalization
        let raw_diff = (target_angle_deg - robot.heading) as i32;
        let norm_diff = verify_angle_normalize(raw_diff.clamp(-540, 540));
        let angle_diff = norm_diff as f64;

        let (target_left, target_right) = match self.state {
            NavState::Turning => {
                if angle_diff.abs() <= self.angle_tolerance_deg {
                    self.state = NavState::Moving;
                    (self.base_speed, self.base_speed)
                } else {
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
                    let raw_steering = (angle_diff * k_p * 100.0) as i32;

                    // Verified differential steering limit
                    let safe_steering = verify_differential_steering(180, raw_steering, 80) as f64 / 100.0;

                    (
                        (self.base_speed - safe_steering).clamp(-2.5, 2.5),
                        (self.base_speed + safe_steering).clamp(-2.5, 2.5),
                    )
                }
            }
            _ => (0.0, 0.0),
        };

        self.ramp_velocities(target_left, target_right, dt)
    }

    fn ramp_velocities(&mut self, target_left: f64, target_right: f64, dt: f64) -> NavCommand {
        let max_step = (self.max_accel * dt * 1000.0).clamp(0.0, 10000.0) as i32;

        // Verified acceleration ramping
        let cur_l = (self.current_left_v * 1000.0).clamp(-10000.0, 10000.0) as i32;
        let tgt_l = (target_left * 1000.0).clamp(-10000.0, 10000.0) as i32;
        self.current_left_v = (verify_accel_ramp(cur_l, tgt_l, max_step) as f64) / 1000.0;

        let cur_r = (self.current_right_v * 1000.0).clamp(-10000.0, 10000.0) as i32;
        let tgt_r = (target_right * 1000.0).clamp(-10000.0, 10000.0) as i32;
        self.current_right_v = (verify_accel_ramp(cur_r, tgt_r, max_step) as f64) / 1000.0;

        NavCommand {
            left_velocity: self.current_left_v,
            right_velocity: self.current_right_v,
        }
    }
}