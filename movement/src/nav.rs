use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

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

#[derive(Debug, Clone, Deserialize)]
pub struct ObjectDetection {
    pub object_position: f64,
    pub object_angle: f64,
    pub found: bool,
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

#[derive(Debug, Clone, Copy, Serialize)]
pub enum MovementState {
    Idle,
    Moving,
    Turning,
    AvoidingLeft,
    AvoidingRight,
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

    // Heading in degrees.
    // 0° = +X direction
    // 90° = +Y direction
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
    
    // -----------------------------------------------------
    // MOVE ROBOT IN SIMULATION
    // -----------------------------------------------------

    pub fn simulate_motion(
        &mut self,
        left_velocity: f64,
        right_velocity: f64,
        dt: f64,
    ) {
        let linear_velocity = (left_velocity + right_velocity) / 2.0;

        let angular_velocity = right_velocity - left_velocity;

        // Simple heading update.
        self.heading += angular_velocity * 30.0 * dt;
        self.heading = normalize_angle(self.heading);

        let heading_rad = degrees_to_radians(self.heading);

        self.position.x +=
            linear_velocity * heading_rad.cos() * dt;

        self.position.y +=
            linear_velocity * heading_rad.sin() * dt;
    }
}

// ---------------------------------------------------------
// MOVEMENT MANAGER
// ---------------------------------------------------------

pub struct MovementManager {
    pub target: Position,

    // Used to remember which side was selected for avoidance.
    avoidance_direction: Option<Direction>,

    // Used when robot is temporarily avoiding an obstacle.
    avoiding: bool,
}

impl MovementManager {
    pub fn new(target: Position) -> Self {
        Self {
            target,
            avoidance_direction: None,
            avoiding: false,
        }
    }

    // -----------------------------------------------------
    // MAIN DECISION FUNCTION
    // -----------------------------------------------------

    pub fn update(
        &mut self,
        robot: &mut Robot,
        detection: &ObjectDetection,
    ) -> MovementCommand {
        // ---------------------------------------------
        // 1. Check whether target has been reached
        // ---------------------------------------------

        let distance_to_target =
            robot.position.distance_to(&self.target);

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

        // ---------------------------------------------
        // 2. Check obstacle
        // ---------------------------------------------

        if detection.found
            && detection.object_position <= SAFETY_DISTANCE
        {
            return self.handle_obstacle(robot, detection);
        }

        // ---------------------------------------------
        // 3. If currently avoiding and obstacle is clear
        // ---------------------------------------------

        if self.avoiding {
            if !detection.found
                || detection.object_position >= OBSTACLE_CLEAR_DISTANCE
            {
                self.avoiding = false;
                self.avoidance_direction = None;
            }
        }

        // ---------------------------------------------
        // 4. Normal target navigation
        // ---------------------------------------------

        self.move_to_target(robot)
    }

    // -----------------------------------------------------
    // OBSTACLE HANDLING
    // -----------------------------------------------------

    fn handle_obstacle(
        &mut self,
        robot: &mut Robot,
        detection: &ObjectDetection,
    ) -> MovementCommand {
        robot.state = MovementState::Stopped;

        /*
            Safety behavior:

            First stop.

            Then decide whether to avoid left or right.

            Positive object angle:
                obstacle is on one side.

            Negative object angle:
                obstacle is on the other side.

            For this first version we select the
            opposite side of the obstacle.
        */

        let direction = if detection.object_angle >= 0.0 {
            Direction::Right
        } else {
            Direction::Left
        };

        self.avoidance_direction = Some(direction);
        self.avoiding = true;

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

    // -----------------------------------------------------
    // MOVE TOWARD TARGET
    // -----------------------------------------------------

    fn move_to_target(
        &mut self,
        robot: &mut Robot,
    ) -> MovementCommand {
        let dx = self.target.x - robot.position.x;
        let dy = self.target.y - robot.position.y;

        let target_angle =
            radians_to_degrees(dy.atan2(dx));

        let heading_error =
            normalize_angle(target_angle - robot.heading);

        /*
            Proportional steering:

                turn = Kp * heading_error

            Positive turn:
                right wheel faster

            Negative turn:
                left wheel faster
        */

        let turn = TURN_GAIN * heading_error;

        let left_velocity =
            clamp(BASE_SPEED - turn);

        let right_velocity =
            clamp(BASE_SPEED + turn);

        let direction;

        if heading_error > 5.0 {
            direction = Direction::Left;
            robot.state = MovementState::Turning;
        } else if heading_error < -5.0 {
            direction = Direction::Right;
            robot.state = MovementState::Turning;
        } else {
            direction = Direction::Forward;
            robot.state = MovementState::Moving;
        }

        MovementCommand {
            state: robot.state,
            direction,
            angle: heading_error,
            left_velocity,
            right_velocity,
        }
    }

    // -----------------------------------------------------
    // STOP COMMAND
    // -----------------------------------------------------

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

// ---------------------------------------------------------
// MATH FUNCTIONS
// ---------------------------------------------------------

fn degrees_to_radians(degrees: f64) -> f64 {
    degrees * PI / 180.0
}

fn radians_to_degrees(radians: f64) -> f64 {
    radians * 180.0 / PI
}

// Normalize angle to [-180, 180)
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