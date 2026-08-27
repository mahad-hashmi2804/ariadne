use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::time::{Duration, Instant};

const MAX_WHEEL_SPEED: f64 = 1.0;
const BASE_SPEED: f64 = 0.7;

const SAFETY_DISTANCE: f64 = 1.0;
const OBSTACLE_CLEAR_DISTANCE: f64 = 1.5;

const POSITION_TOLERANCE: f64 = 0.15;
const TURN_GAIN: f64 = 0.015;

// ---------------------------------------------------------
// BASIC DATA TYPES
// ---------------------------------------------------------

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Position {
    pub x: f64,
    pub y: f64,
}

impl Position {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn distance_to(&self, other: &Position) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        (dx * dx + dy * dy).sqrt()
    }
}

// ---------------------------------------------------------
// OBJECT DETECTION INPUT
// ---------------------------------------------------------

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct ObjectDetection {
    #[serde(alias = "obstacle_detected")]
    pub found: bool,

    #[serde(alias = "distance_m")]
    pub object_position: f64,

    #[serde(alias = "angle_deg")]
    pub object_angle: f64,
}

impl Default for ObjectDetection {
    fn default() -> Self {
        Self {
            found: false,
            object_position: 999.0,
            object_angle: 0.0,
        }
    }
}

// ---------------------------------------------------------
// MOVEMENT ENUMS
// ---------------------------------------------------------

#[derive(Debug, Clone, Copy, Serialize)]
pub enum Direction {
    Forward,
    Backward,
    Left,
    Right,
    Stop,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq)]
pub enum MovementState {
    Idle,
    Moving,
    Turning,
    AvoidingLeft,
    AvoidingRight,
    RecoveryBackup,
    Stopped,
    Reached,
}

// ---------------------------------------------------------
// MOVEMENT COMMAND
// ---------------------------------------------------------

#[derive(Debug, Serialize)]
pub struct MovementCommand {
    pub state: MovementState,
    pub direction: Direction,
    pub angle: f64,
    pub left_velocity: f64,
    pub right_velocity: f64,
}

// ---------------------------------------------------------
// ROBOT
// ---------------------------------------------------------

pub struct Robot {
    pub position: Position,
    pub heading: f64,
    pub state: MovementState,
}

impl Robot {
    pub fn new(position: Position, heading: f64) -> Self {
        Self {
            position,
            heading: normalize_angle(heading),
            state: MovementState::Idle,
        }
    }

    pub fn simulate_motion(
        &mut self,
        left_velocity: f64,
        right_velocity: f64,
        dt: f64,
    ) {
        let linear_velocity = (left_velocity + right_velocity) / 2.0;
        let angular_velocity = right_velocity - left_velocity;

        self.heading += angular_velocity * 30.0 * dt;
        self.heading = normalize_angle(self.heading);

        let heading_rad = degrees_to_radians(self.heading);

        self.position.x += linear_velocity * heading_rad.cos() * dt;
        self.position.y += linear_velocity * heading_rad.sin() * dt;
    }
}

// ---------------------------------------------------------
// MOVEMENT MANAGER
// ---------------------------------------------------------

pub struct MovementManager {
    pub target: Position,
    avoidance_direction: Option<Direction>,
    avoiding: bool,

    // Timing & State locks
    state_entry_time: Instant,
    avoidance_start_time: Option<Instant>,
    recovery_start_time: Option<Instant>,

    min_state_duration: Duration,    // Prevent chatter (500ms)
    max_avoidance_timeout: Duration, // Timeout turning after 3s
    recovery_duration: Duration,     // Run backup maneuver for 1.5s
}

impl MovementManager {
    pub fn new(target: Position) -> Self {
        let now = Instant::now();
        Self {
            target,
            avoidance_direction: None,
            avoiding: false,
            state_entry_time: now,
            avoidance_start_time: None,
            recovery_start_time: None,
            min_state_duration: Duration::from_millis(500),
            max_avoidance_timeout: Duration::from_secs(3),
            recovery_duration: Duration::from_millis(1500),
        }
    }

    pub fn update(
        &mut self,
        robot: &mut Robot,
        detection: &ObjectDetection,
    ) -> MovementCommand {
        let now = Instant::now();
        let time_in_state = now.duration_since(self.state_entry_time);

        // 1. Target Reached check
        let distance_to_target = robot.position.distance_to(&self.target);
        if distance_to_target <= POSITION_TOLERANCE {
            robot.state = MovementState::Reached;
            return MovementCommand {
                state: MovementState::Reached,
                direction: Direction::Stop,
                angle: 0.0,
                left_velocity: 0.0,
                right_velocity: 0.0,
            };
        }

        // 2. Active Recovery Routine Execution
        if let Some(rec_start) = self.recovery_start_time {
            if now.duration_since(rec_start) < self.recovery_duration {
                robot.state = MovementState::RecoveryBackup;
                return MovementCommand {
                    state: MovementState::RecoveryBackup,
                    direction: Direction::Backward,
                    angle: 0.0,
                    left_velocity: -0.4,
                    right_velocity: -0.4,
                };
            } else {
                // Recovery complete -> Full timer & flag reset
                self.recovery_start_time = None;
                self.avoidance_start_time = None;
                self.avoiding = false;
                self.avoidance_direction = None;
            }
        }

        // 3. Trigger Recovery if stuck turning too long (> 3.0s)
        if let Some(start_time) = self.avoidance_start_time {
            if now.duration_since(start_time) >= self.max_avoidance_timeout {
                self.recovery_start_time = Some(now);
                self.avoidance_start_time = None; // Clear to break infinite loop
                robot.state = MovementState::RecoveryBackup;
                return MovementCommand {
                    state: MovementState::RecoveryBackup,
                    direction: Direction::Backward,
                    angle: 0.0,
                    left_velocity: -0.4,
                    right_velocity: -0.4,
                };
            }
        }

        // 4. Enforce Hysteresis: Hold state for minimum lock duration
        if time_in_state < self.min_state_duration && self.avoiding {
            return self.maintain_current_avoidance(robot);
        }

        // 5. Handle Obstacle Detection
        if detection.found && detection.object_position <= SAFETY_DISTANCE {
            return self.handle_obstacle(robot, detection, now);
        }

        // 6. Clear Avoidance if path is clear
        if self.avoiding {
            if !detection.found || detection.object_position >= OBSTACLE_CLEAR_DISTANCE {
                self.avoiding = false;
                self.avoidance_direction = None;
                self.avoidance_start_time = None;
            }
        }

        // 7. Normal Navigation
        self.move_to_target(robot)
    }

    fn handle_obstacle(
        &mut self,
        robot: &mut Robot,
        detection: &ObjectDetection,
        now: Instant,
    ) -> MovementCommand {
        if self.avoidance_start_time.is_none() {
            self.avoidance_start_time = Some(now);
        }

        let direction = if detection.object_angle >= 0.0 {
            Direction::Right
        } else {
            Direction::Left
        };

        self.avoidance_direction = Some(direction);
        self.avoiding = true;
        self.state_entry_time = now;

        match direction {
            Direction::Left => {
                robot.state = MovementState::AvoidingLeft;
                MovementCommand {
                    state: MovementState::AvoidingLeft,
                    direction: Direction::Left,
                    angle: 30.0,
                    left_velocity: 0.35,
                    right_velocity: 0.75,
                }
            }
            Direction::Right => {
                robot.state = MovementState::AvoidingRight;
                MovementCommand {
                    state: MovementState::AvoidingRight,
                    direction: Direction::Right,
                    angle: -30.0,
                    left_velocity: 0.75,
                    right_velocity: 0.35,
                }
            }
            _ => self.stop_command(),
        }
    }

    fn maintain_current_avoidance(&self, robot: &mut Robot) -> MovementCommand {
        match self.avoidance_direction {
            Some(Direction::Left) => {
                robot.state = MovementState::AvoidingLeft;
                MovementCommand {
                    state: MovementState::AvoidingLeft,
                    direction: Direction::Left,
                    angle: 30.0,
                    left_velocity: 0.35,
                    right_velocity: 0.75,
                }
            }
            Some(Direction::Right) => {
                robot.state = MovementState::AvoidingRight;
                MovementCommand {
                    state: MovementState::AvoidingRight,
                    direction: Direction::Right,
                    angle: -30.0,
                    left_velocity: 0.75,
                    right_velocity: 0.35,
                }
            }
            _ => self.stop_command(),
        }
    }

    fn move_to_target(&mut self, robot: &mut Robot) -> MovementCommand {
        let dx = self.target.x - robot.position.x;
        let dy = self.target.y - robot.position.y;

        let target_angle = radians_to_degrees(dy.atan2(dx));
        let heading_error = normalize_angle(target_angle - robot.heading);

        let turn = TURN_GAIN * heading_error;
        let left_velocity = clamp(BASE_SPEED - turn);
        let right_velocity = clamp(BASE_SPEED + turn);

        let direction = if heading_error > 5.0 {
            robot.state = MovementState::Turning;
            Direction::Left
        } else if heading_error < -5.0 {
            robot.state = MovementState::Turning;
            Direction::Right
        } else {
            robot.state = MovementState::Moving;
            Direction::Forward
        };

        MovementCommand {
            state: robot.state,
            direction,
            angle: heading_error,
            left_velocity,
            right_velocity,
        }
    }

    fn stop_command(&self) -> MovementCommand {
        MovementCommand {
            state: MovementState::Stopped,
            direction: Direction::Stop,
            angle: 0.0,
            left_velocity: 0.0,
            right_velocity: 0.0,
        }
    }
}

fn degrees_to_radians(degrees: f64) -> f64 {
    degrees * PI / 180.0
}

fn radians_to_degrees(radians: f64) -> f64 {
    radians * 180.0 / PI
}

fn normalize_angle(mut angle: f64) -> f64 {
    while angle >= 180.0 {
        angle -= 360.0;
    }
    while angle < -180.0 {
        angle += 360.0;
    }
    angle
}

fn clamp(value: f64) -> f64 {
    value.clamp(-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
}