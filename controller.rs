// =====================================================================
// TEAM 3 MODULE — Converts Team 1's navigation output + Team 2's
// obstacle-avoidance decision into final wheel velocities.
//
// HOW TO USE THIS FILE:
//   1. Save this as movement/src/controller.rs in the repo.
//   2. Add `mod controller;` near the top of movement/src/main.rs.
//   3. Once Team 1 and Team 2 confirm their real struct shapes, update
//      the two "INPUT" structs below to match exactly what they send.
//
// This file does NOT depend on Team 1 or Team 2's actual code — it
// only depends on the *shapes* (struct fields) from the task doc.
// That's the whole point: you only need to agree on field names and
// types, not on how their code works internally.
// =====================================================================

// ---------------------------------------------------------------
// INPUT #1 — what Team 1 (navigation) is supposed to send you
// ---------------------------------------------------------------
#[derive(Debug, Clone, Copy)]
pub struct NavCommand {
    pub distance: f64,       // how far the robot is from its target, in meters
    pub target_angle: f64,   // the angle (degrees) pointing straight at the target
    pub heading_angle: f64,  // the angle (degrees) the robot should currently steer toward
}

// ---------------------------------------------------------------
// INPUT #2 — what Team 2 (obstacle avoidance) is supposed to send you
// ---------------------------------------------------------------
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Direction {
    Left,
    Right,
    None, // no avoidance needed
}

#[derive(Debug, Clone, Copy)]
pub struct AvoidanceDecision {
    pub obstacle_detected: bool,
    pub avoid: bool,          // true = robot needs to actively steer around something
    pub direction: Direction, // which way to turn to avoid it
    pub angle: f64,           // how sharply to turn (degrees)
}

// ---------------------------------------------------------------
// OUTPUT — what YOU (Team 3) are responsible for producing
// ---------------------------------------------------------------
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelCommand {
    pub left_velocity: f64,
    pub right_velocity: f64,
}

// Tuning constants — safe to tweak later once you test with the real robot/sim
const MAX_WHEEL_SPEED: f64 = 1.0; // wheels are never allowed to exceed this speed
const BASE_SPEED: f64 = 0.7;      // normal forward speed when nothing is wrong
const TURN_GAIN: f64 = 0.015;     // how strongly heading error affects turning

// ---------------------------------------------------------------
// THE MAIN FUNCTION — this is your actual Team 3 deliverable
// ---------------------------------------------------------------
//
// Beginner note: this function takes no ownership of anything else's
// code. It just reads two structs and returns a third. That's the
// entire "connection" between teams — nothing more complicated than
// function arguments and a return value.
pub fn fuse_to_wheels(nav: &NavCommand, avoidance: &AvoidanceDecision) -> WheelCommand {
    // ---- ERROR HANDLING FIRST ----
    // Real sensors can send garbage data (NaN, infinity, wild numbers).
    // Never trust incoming data blindly — always check it first.
    if !nav.heading_angle.is_finite() || !nav.distance.is_finite() {
        // If navigation data is broken, the safest thing to do is stop.
        return WheelCommand { left_velocity: 0.0, right_velocity: 0.0 };
    }

    if !avoidance.angle.is_finite() {
        // If avoidance data is broken, ignore avoidance and treat as "no obstacle".
        return navigate_normally(nav);
    }

    // ---- DECISION LOGIC ----
    // Obstacle avoidance always takes priority over normal navigation,
    // because avoiding a collision matters more than reaching the target
    // on the most direct path.
    if avoidance.obstacle_detected && avoidance.avoid {
        return avoid_obstacle(avoidance);
    }

    // No obstacle problem right now — just steer toward the target.
    navigate_normally(nav)
}

// Handles the "no obstacle in the way" case.
fn navigate_normally(nav: &NavCommand) -> WheelCommand {
    // heading_angle here represents how far off-course we currently are.
    // Positive = need to turn one way, negative = need to turn the other way.
    let turn = TURN_GAIN * nav.heading_angle;

    let left = clamp_speed(BASE_SPEED - turn);
    let right = clamp_speed(BASE_SPEED + turn);

    WheelCommand { left_velocity: left, right_velocity: right }
}

// Handles the "obstacle in the way, need to avoid it" case.
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

// Keeps a wheel speed within the safe, allowed range.
fn clamp_speed(value: f64) -> f64 {
    value.clamp(-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
}

// =====================================================================
// TESTS — you can run these right now with `cargo test`, with zero
// hardware and zero dependency on Team 1 or Team 2's real code.
// This is exactly how you prove your part works before integration.
// =====================================================================
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
        // Should be turning, not driving straight
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
