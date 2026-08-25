
// mov.rs
//
// All movement and navigation logic is here.

use std::f64::consts::PI;

// =====================================================
// DATA STRUCTURES
// =====================================================

#[derive(Debug, Clone, Copy)]
pub struct Position {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64, // Robot heading in radians
}

#[derive(Debug, Clone, Copy)]
pub struct Obstacle {
    pub ob_found: bool,
    pub ob_distance: f64, // meters
    pub ob_angle: f64,    // degrees
}

#[derive(Debug, Clone, Copy)]
pub struct MotionCommand {
    pub linear_velocity: f64,  // m/s
    pub angular_velocity: f64, // rad/s
}

#[derive(Debug, Clone, Copy)]
pub enum MovementState {
    MovingToTarget,
    ObstacleDetected,
    AvoidingLeft,
    AvoidingRight,
    TargetReached,
    Stopped,
}

// =====================================================
// CONSTANTS
// =====================================================

const SAFE_DISTANCE: f64 = 0.8;
const POSITION_TOLERANCE: f64 = 0.15;
const HEADING_TOLERANCE: f64 = 0.1;

const FORWARD_SPEED: f64 = 0.5;
const TURN_SPEED: f64 = 0.8;

// =====================================================
// POSITION FUNCTIONS
// =====================================================

pub fn calculate_distance(
    current: Position,
    target: Position,
) -> f64 {
    let dx = target.x - current.x;
    let dy = target.y - current.y;

    (dx * dx + dy * dy).sqrt()
}

pub fn calculate_target_angle(
    current: Position,
    target: Position,
) -> f64 {
    let dx = target.x - current.x;
    let dy = target.y - current.y;

    dy.atan2(dx)
}

// =====================================================
// ANGLE FUNCTIONS
// =====================================================

pub fn normalize_angle(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= 2.0 * PI;
    }

    while angle < -PI {
        angle += 2.0 * PI;
    }

    angle
}

pub fn calculate_heading_error(
    current_heading: f64,
    target_heading: f64,
) -> f64 {
    normalize_angle(target_heading - current_heading)
}

// =====================================================
// BASIC MOVEMENT
// =====================================================

pub fn forward() -> MotionCommand {
    MotionCommand {
        linear_velocity: FORWARD_SPEED,
        angular_velocity: 0.0,
    }
}

pub fn backward() -> MotionCommand {
    MotionCommand {
        linear_velocity: -FORWARD_SPEED,
        angular_velocity: 0.0,
    }
}

pub fn turn_left() -> MotionCommand {
    MotionCommand {
        linear_velocity: 0.0,
        angular_velocity: TURN_SPEED,
    }
}

pub fn turn_right() -> MotionCommand {
    MotionCommand {
        linear_velocity: 0.0,
        angular_velocity: -TURN_SPEED,
    }
}

pub fn stop() -> MotionCommand {
    MotionCommand {
        linear_velocity: 0.0,
        angular_velocity: 0.0,
    }
}

// =====================================================
// TARGET CHECK
// =====================================================

pub fn target_reached(
    current: Position,
    target: Position,
) -> bool {
    calculate_distance(current, target)
        <= POSITION_TOLERANCE
}

// =====================================================
// OBSTACLE CHECK
// =====================================================

pub fn obstacle_is_close(
    obstacle: Obstacle,
) -> bool {
    obstacle.ob_found
        && obstacle.ob_distance <= SAFE_DISTANCE
}


pub fn choose_avoidance_direction(
    obstacle: Obstacle,
) -> MovementState {

    if obstacle.ob_angle > 0.0 {
        // Obstacle is on the left.
        // Avoid by going right.
        MovementState::AvoidingRight
    } else {
        // Obstacle is on the right or directly ahead.
        // Avoid by going left.
        MovementState::AvoidingLeft
    }
}

// =====================================================
// OBSTACLE AVOIDANCE
// =====================================================

pub fn avoid_obstacle(
    obstacle: Obstacle,
) -> (MovementState, MotionCommand) {

    let direction =
        choose_avoidance_direction(obstacle);

    match direction {
        MovementState::AvoidingLeft => {
            (
                MovementState::AvoidingLeft,
                turn_left(),
            )
        }

        MovementState::AvoidingRight => {
            (
                MovementState::AvoidingRight,
                turn_right(),
            )
        }

        _ => {
            (
                MovementState::Stopped,
                stop(),
            )
        }
    }
}

// =====================================================
// MAIN MOVEMENT DECISION
// =====================================================

pub fn decide_movement(
    current_pose: Pose,
    target: Position,
    obstacle: Obstacle,
) -> (MovementState, MotionCommand) {

    let current_position = Position {
        x: current_pose.x,
        y: current_pose.y,
    };

    // -------------------------------------------------
    // 1. Check whether target has been reached
    // -------------------------------------------------

    if target_reached(
        current_position,
        target,
    ) {
        return (
            MovementState::TargetReached,
            stop(),
        );
    }

    // -------------------------------------------------
    // 2. Check obstacle
    // -------------------------------------------------

    if obstacle_is_close(obstacle) {
        return avoid_obstacle(obstacle);
    }

    // -------------------------------------------------
    // 3. Calculate direction toward target
    // -------------------------------------------------

    let target_angle =
        calculate_target_angle(
            current_position,
            target,
        );

    // -------------------------------------------------
    // 4. Calculate heading error
    // -------------------------------------------------

    let heading_error =
        calculate_heading_error(
            current_pose.theta,
            target_angle,
        );

    // -------------------------------------------------
    // 5. Turn toward target
    // -------------------------------------------------

    if heading_error > HEADING_TOLERANCE {
        return (
            MovementState::MovingToTarget,
            turn_left(),
        );
    }

    if heading_error < -HEADING_TOLERANCE {
        return (
            MovementState::MovingToTarget,
            turn_right(),
        );
    }

    // -------------------------------------------------
    // 6. Robot is facing target
    // -------------------------------------------------

    (
        MovementState::MovingToTarget,
        forward(),
    )
}
