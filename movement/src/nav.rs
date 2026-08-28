use std::f64::consts::PI;
use std::time::Duration;
use crate::types::{NavCommand, NavState, ObstacleFrame, Point2D, RobotPose};

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

        let has_active_obstacle = self.obstacle.detected
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
                    self.bypass_target_dist = (self.last_obstacle_dist + 0.5).max(1.0);
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

        let mut angle_diff = target_angle_deg - robot.heading;
        while angle_diff > 180.0 { angle_diff -= 360.0; }
        while angle_diff < -180.0 { angle_diff += 360.0; }

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