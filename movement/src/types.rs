//! # Movement Subsystem Shared Types
//!
//! Defines 2D geometric points, robot pose, IMU telemetry packets, obstacle frames,
//! state machine enums, and motor command structures.

use std::time::Instant;

/// Represents a 2D planar position coordinate in world space (meters).
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

/// Represents the physical pose of the robot in world coordinates.
#[derive(Debug, Default, Clone, Copy)]
pub struct RobotPose {
    pub position: Point2D,
    /// Heading angle in degrees bounded within [-180.0, 180.0].
    pub heading: f64,
}

/// Raw 52-byte binary telemetry frame received from MuJoCo simulator over UDP (100 Hz).
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
    /// Decodes a 52-byte little-endian byte array into thirteen 32-bit floating point numbers.
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

/// Filtered obstacle telemetry frame parsed from the `detection` module JSON payloads.
#[derive(Debug, Default, Clone, Copy)]
pub struct ObstacleFrame {
    pub detected: bool,
    pub distance_m: f64,
    pub angle_deg: f64,
    pub last_seen: Option<Instant>,
}

/// Operational state enum for the navigation manager state machine.
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum NavState {
    Idle,
    Turning,
    Moving,
    AvoidingTurn,
    AvoidingBypass,
    Reached,
}

/// Motor track command output dispatched to simulator over UDP.
#[derive(Debug, Clone, Copy)]
pub struct NavCommand {
    pub left_velocity: f64,
    pub right_velocity: f64,
}