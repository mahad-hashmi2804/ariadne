use vstd::prelude::*;

verus! {

// =====================================================================
// PART 1: STATEFUL CONTROLLER VERIFICATION (4 Proofs)
// =====================================================================

pub struct NavController {
    pub state: u32, // 0: Idle, 1: Turning, 2: Moving, 3: AvoidingTurn, 4: AvoidingBypass, 5: Reached
    pub current_left_v: i32,
    pub current_right_v: i32,
    pub max_accel_step: i32,
    pub bypass_target_dist_mm: i32,
}

impl NavController {
    /// 1. Formally proves constructor initializes safe default state
    pub fn new(max_accel_step: i32) -> (s: Self)
        requires
            max_accel_step >= 0 && max_accel_step <= 10000,
        ensures
            s.state == 0,
            s.current_left_v == 0,
            s.current_right_v == 0,
            s.max_accel_step == max_accel_step,
            s.bypass_target_dist_mm == 0,
    {
        NavController {
            state: 0,
            current_left_v: 0,
            current_right_v: 0,
            max_accel_step,
            bypass_target_dist_mm: 0,
        }
    }

    /// 2. Formally proves state transitions out of IDLE (0) are restricted
    pub fn set_state(&mut self, target_state: u32) -> (is_valid: bool)
        ensures
            (old(self).state == 0) ==> (is_valid == (target_state == 0 || target_state == 1 || target_state == 5)),
            is_valid ==> (final(self).state == target_state),
            !is_valid ==> (final(self).state == old(self).state),
    {
        if self.state == 0 {
            if target_state == 0 || target_state == 1 || target_state == 5 {
                self.state = target_state;
                true
            } else {
                false
            }
        } else {
            self.state = target_state;
            true
        }
    }

    /// 3. Formally proves mutating velocities via ramp_velocities respects acceleration bounds
    pub fn ramp_velocities(&mut self, target_left: i32, target_right: i32)
        requires
            old(self).max_accel_step >= 0 && old(self).max_accel_step <= 10000,
            old(self).current_left_v >= -10000 && old(self).current_left_v <= 10000,
            old(self).current_right_v >= -10000 && old(self).current_right_v <= 10000,
            target_left >= -10000 && target_left <= 10000,
            target_right >= -10000 && target_right <= 10000,
        ensures
            final(self).current_left_v - old(self).current_left_v <= old(self).max_accel_step,
            old(self).current_left_v - final(self).current_left_v <= old(self).max_accel_step,
            final(self).current_right_v - old(self).current_right_v <= old(self).max_accel_step,
            old(self).current_right_v - final(self).current_right_v <= old(self).max_accel_step,
    {
        let diff_l = target_left - self.current_left_v;
        let step_l = if diff_l > self.max_accel_step {
            self.max_accel_step
        } else if diff_l < -self.max_accel_step {
            -self.max_accel_step
        } else {
            diff_l
        };
        self.current_left_v = self.current_left_v + step_l;

        let diff_r = target_right - self.current_right_v;
        let step_r = if diff_r > self.max_accel_step {
            self.max_accel_step
        } else if diff_r < -self.max_accel_step {
            -self.max_accel_step
        } else {
            diff_r
        };
        self.current_right_v = self.current_right_v + step_r;
    }

    /// 4. Formally proves updating bypass distance strictly overshoots geometry
    pub fn update_bypass_distance(&mut self, obstacle_depth_mm: i32, safety_buffer_mm: i32)
        requires
            obstacle_depth_mm > 0 && obstacle_depth_mm <= 10000,
            safety_buffer_mm >= 400 && safety_buffer_mm <= 2000,
        ensures
            final(self).bypass_target_dist_mm > obstacle_depth_mm,
            final(self).bypass_target_dist_mm >= obstacle_depth_mm + safety_buffer_mm,
    {
        self.bypass_target_dist_mm = obstacle_depth_mm + safety_buffer_mm;
    }
}

// =====================================================================
// PART 2: PURE FUNCTIONAL ALGORITHM VERIFICATION (6 Proofs)
// =====================================================================

/// 5. Formally proves angle normalization stays within [-180°, 180°]
pub fn verify_angle_normalize(angle: i32) -> (norm: i32)
    requires
        angle >= -540 && angle <= 540,
    ensures
        norm >= -180,
        norm <= 180,
        (angle - norm) % 360 == 0,
{
    let mut a = angle;
    if a > 180 {
        a = a - 360;
    }
    if a < -180 {
        a = a + 360;
    }
    a
}

/// 6. Formally proves raw depth camera readings are filtered to valid bounds
pub fn verify_depth_threshold(depth_mm: i32, min_mm: i32, max_mm: i32) -> (is_valid: bool)
    requires
        min_mm > 0,
        max_mm > min_mm,
    ensures
        is_valid ==> (depth_mm >= min_mm && depth_mm <= max_mm),
{
    depth_mm >= min_mm && depth_mm <= max_mm
}

/// 7. Formally proves differential steering offset never exceeds max track split
pub fn verify_differential_steering(base_v: i32, steering: i32, max_split: i32) -> (clamped_steering: i32)
    requires
        max_split > 0,
    ensures
        clamped_steering <= max_split,
        clamped_steering >= -max_split,
{
    if steering > max_split {
        max_split
    } else if steering < -max_split {
        -max_split
    } else {
        steering
    }
}

/// 8. Formally proves circuit waypoint indexing can never overflow array bounds
pub fn verify_circuit_index_advance(current_idx: usize, circuit_len: usize) -> (next_idx: usize)
    requires
        circuit_len > 0,
        current_idx < circuit_len,
    ensures
        next_idx < circuit_len,
{
    (current_idx + 1) % circuit_len
}

/// 9. Formally proves calibration bias mean calculation over 50 samples is safe
pub fn verify_calibration_mean(sum_gyro_z: u32) -> (mean: u32)
    requires
        sum_gyro_z <= 50000,
    ensures
        mean == sum_gyro_z / 50,
{
    sum_gyro_z / 50
}

/// 10. Formally proves IMU 52-byte packet chunk slicing never exceeds buffer boundaries
pub fn verify_imu_slice_offset(chunk_index: usize) -> (offset: usize)
    requires
        chunk_index < 13,
    ensures
        offset <= 48,
        offset + 4 <= 52,
{
    chunk_index * 4
}

} // verus!
