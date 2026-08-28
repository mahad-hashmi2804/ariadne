// =====================================================================
// TEAM 3 MODULE — Converts Team 1's navigation output + Team 2's
// obstacle-avoidance decision into final wheel velocities.
// =====================================================================

#[derive(Debug, Clone, Copy)]
pub struct NavCommand {
    pub distance: f64,
    pub target_angle: f64,
    pub heading_angle: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Direction {
    Left,
    Right,
    None,
}

#[derive(Debug, Clone, Copy)]
pub struct AvoidanceDecision {
    pub obstacle_detected: bool,
    pub avoid: bool,
    pub direction: Direction,
    pub angle: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelCommand {
    pub left_velocity: f64,
    pub right_velocity: f64,
}

const MAX_WHEEL_SPEED: f64 = 1.0;
const BASE_SPEED: f64 = 0.7;
const TURN_GAIN: f64 = 0.015;

pub fn fuse_to_wheels(nav: &NavCommand, avoidance: &AvoidanceDecision) -> WheelCommand {
    if !nav.heading_angle.is_finite() || !nav.distance.is_finite() {
        return WheelCommand { left_velocity: 0.0, right_velocity: 0.0 };
    }

    if !avoidance.angle.is_finite() {
        return navigate_normally(nav);
    }

    if avoidance.obstacle_detected && avoidance.avoid {
        return avoid_obstacle(avoidance);
    }

    navigate_normally(nav)
}

fn navigate_normally(nav: &NavCommand) -> WheelCommand {
    let turn = TURN_GAIN * nav.heading_angle;
    let left = clamp_speed(BASE_SPEED - turn);
    let right = clamp_speed(BASE_SPEED + turn);
    WheelCommand { left_velocity: left, right_velocity: right }
}

fn avoid_obstacle(avoidance: &AvoidanceDecision) -> WheelCommand {
    match avoidance.direction {
        Direction::Left => WheelCommand {
            left_velocity: clamp_speed(BASE_SPEED * 0.5),
            right_velocity: clamp_speed(BASE_SPEED * 1.1),
        },
        Direction::Right => WheelCommand {
            left_velocity: clamp_speed(BASE_SPEED * 1.1),
            right_velocity: clamp_speed(BASE_SPEED * 0.5),
        },
        Direction::None => WheelCommand { left_velocity: 0.0, right_velocity: 0.0 },
    }
}

fn clamp_speed(value: f64) -> f64 {
    value.clamp(-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
}

fn main() {
    let nav = NavCommand {
        distance: 5.0,
        target_angle: 0.0,
        heading_angle: 0.0,
    };

    let avoidance = AvoidanceDecision {
        obstacle_detected: false,
        avoid: false,
        direction: Direction::None,
        angle: 0.0,
    };

    let wheels = fuse_to_wheels(&nav, &avoidance);
    println!("No obstacle case -> {:?}", wheels);

    let avoidance_with_obstacle = AvoidanceDecision {
        obstacle_detected: true,
        avoid: true,
        direction: Direction::Left,
        angle: 30.0,
    };

    let wheels2 = fuse_to_wheels(&nav, &avoidance_with_obstacle);
    println!("Obstacle on left -> {:?}", wheels2);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drives_straight_when_on_target_heading() {
        let nav = NavCommand { distance: 5.0, target_angle: 0.0, heading_angle: 0.0 };
        let avoidance = AvoidanceDecision {
            obstacle_detected: false,
            avoid: false,
            direction: Direction::None,
            angle: 0.0,
        };
        let wheels = fuse_to_wheels(&nav, &avoidance);
        assert_eq!(wheels, WheelCommand { left_velocity: BASE_SPEED, right_velocity: BASE_SPEED });
    }

    #[test]
    fn avoidance_overrides_normal_navigation() {
        let nav = NavCommand { distance: 5.0, target_angle: 0.0, heading_angle: 0.0 };
        let avoidance = AvoidanceDecision {
            obstacle_detected: true,
            avoid: true,
            direction: Direction::Left,
            angle: 30.0,
        };
        let wheels = fuse_to_wheels(&nav, &avoidance);
        assert_ne!(wheels.left_velocity, wheels.right_velocity);
    }

    #[test]
    fn stops_safely_on_broken_nav_data() {
        let nav = NavCommand { distance: f64::NAN, target_angle: 0.0, heading_angle: 0.0 };
        let avoidance = AvoidanceDecision {
            obstacle_detected: false,
            avoid: false,
            direction: Direction::None,
            angle: 0.0,
        };
        let wheels = fuse_to_wheels(&nav, &avoidance);
        assert_eq!(wheels, WheelCommand { left_velocity: 0.0, right_velocity: 0.0 });
    }

    #[test]
    fn wheel_speed_never_exceeds_max() {
        let nav = NavCommand { distance: 5.0, target_angle: 0.0, heading_angle: 500.0 };
        let avoidance = AvoidanceDecision {
            obstacle_detected: false,
            avoid: false,
            direction: Direction::None,
            angle: 0.0,
        };
        let wheels = fuse_to_wheels(&nav, &avoidance);
        assert!(wheels.left_velocity.abs() <= MAX_WHEEL_SPEED);
        assert!(wheels.right_velocity.abs() <= MAX_WHEEL_SPEED);
    }
}
