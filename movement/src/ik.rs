// src/ik.rs

/// Represents the output velocities for both tracks.
pub struct TrackVelocities {
    pub v_left: f64,
    pub v_right: f64,
}

/// Calculates left and right track velocities from desired linear velocity (v),
/// angular velocity (omega), and track width (W).
pub fn calculate_track_velocities(v: f64, omega: f64, track_width: f64) -> TrackVelocities {
    let v_left = v - (omega * track_width / 2.0);
    let v_right = v + (omega * track_width / 2.0);

    TrackVelocities { v_left, v_right }
}