use std::f64::consts::PI;

/// TrackKinematics handles translation between high-level displacement/twist commands
/// and concrete low-level wheel/track velocities streamed to actuators.
pub struct TrackKinematics {
    pub track_width: f64,   // Distance between left and right track centers (m)
    pub wheel_radius: f64,  // Drive wheel radius (m)
    pub max_speed: f64,     // Maximum linear speed limit per track (m/s)
}

impl TrackKinematics {
    pub fn new(track_width: f64, wheel_radius: f64, max_speed: f64) -> Self {
        Self {
            track_width,
            wheel_radius,
            max_speed,
        }
    }

    /// Converts body linear velocity (m/s) and angular velocity (rad/s) into (v_left, v_right)
    pub fn twist_to_wheel_speeds(&self, linear_v: f64, angular_w: f64) -> (f64, f64) {
        let v_left = linear_v - (angular_w * self.track_width / 2.0);
        let v_right = linear_v + (angular_w * self.track_width / 2.0);

        (
            v_left.clamp(-self.max_speed, self.max_speed),
            v_right.clamp(-self.max_speed, self.max_speed),
        )
    }

    /// Converts high-level "move X meters" command into track target speeds.
    pub fn move_distance(&self, distance_m: f64, cruise_speed: f64) -> (f64, f64) {
        let sign = if distance_m >= 0.0 { 1.0 } else { -1.0 };
        let target_v = (cruise_speed.abs() * sign).clamp(-self.max_speed, self.max_speed);

        // Straight translation: Both tracks move at identical speeds
        (target_v, target_v)
    }

    /// Converts high-level "turn Y degrees" command into in-place pivot track speeds.
    /// Positive degrees = Counter-Clockwise (Left turn)
    /// Negative degrees = Clockwise (Right turn)
    pub fn turn_degrees(&self, degrees: f64, turn_speed: f64) -> (f64, f64) {
        let rad = degrees * PI / 180.0;
        let sign = if rad >= 0.0 { 1.0 } else { -1.0 };
        let speed = (turn_speed.abs() * sign).clamp(-self.max_speed, self.max_speed);

        // In-place differential pivot: Left track reverses, right track moves forward
        let v_left = -speed;
        let v_right = speed;

        (v_left, v_right)
    }

    /// Packs (v_left, v_right) into the 16-byte Little-Endian buffer required by MuJoCo UDP Port 5555
    pub fn serialize_actuator_bytes(&self, v_left: f64, v_right: f64) -> [u8; 16] {
        let mut buffer = [0u8; 16];
        buffer[0..8].copy_from_slice(&v_left.to_le_bytes());
        buffer[8..16].copy_from_slice(&v_right.to_le_bytes());
        buffer
    }
}