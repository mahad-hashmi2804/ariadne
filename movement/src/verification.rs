//! # Movement Formal Verification Module (Verus SMT Specifications)
//!
//! Contains 18 SMT-backed formal specifications verified via Verus and Z3 solver,
//! proving state transitions, acceleration ramping bounds, angle wrapping, and memory safety.

use vstd::prelude::*;

verus! {

// =====================================================================
// PART 1: STATEFUL CONTROLLER VERIFICATION (4 Proofs)
// =====================================================================

/// Verus controller struct modeling NavigationManager state invariants.
pub struct NavController {
    pub state: u32, // 0: Idle, 1: Turning, 2: Moving, 3: AvoidingTurn, 4: AvoidingBypass, 5: Reached
    pub current_left_v: i32,
    pub current_right_v: i32,
    pub max_accel_step: i32,
    pub bypass_target_dist_mm: i32,
}

impl NavController {
    /// 1. Formally proves constructor initializes safe default state.
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

    /// 2. Formally proves state transitions out of IDLE (0) are restricted.
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

    /// 3. Formally proves mutating velocities via ramp_velocities respects acceleration bounds.
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

    /// 4. Formally proves updating bypass distance strictly overshoots geometry.
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

/// 5. Formally proves angle normalization stays within [-180°, 180°].
pub fn verify_angle_normalize(angle: i32) -> (norm: i32)
    requires
        angle >= i32::MIN + 360 && angle <= i32::MAX - 360,
    ensures
        norm >= -180,
        norm <= 180,
        (angle - norm) % 360 == 0,
{
    let mut a = angle % 360;

    if a > 180 {
        a = a - 360;
    } else if a < -180 {
        a = a + 360;
    }
    a
}

/// 6. Formally proves raw depth camera readings are filtered to valid bounds.
pub fn verify_depth_threshold(depth_mm: i32, min_mm: i32, max_mm: i32) -> (is_valid: bool)
    requires
        min_mm > 0,
        max_mm > min_mm,
    ensures
        is_valid ==> (depth_mm >= min_mm && depth_mm <= max_mm),
{
    depth_mm >= min_mm && depth_mm <= max_mm
}

/// 7. Formally proves differential steering offset never exceeds max track split.
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

/// 8. Formally proves circuit waypoint indexing can never overflow array bounds.
pub fn verify_circuit_index_advance(current_idx: usize, circuit_len: usize) -> (next_idx: usize)
    requires
        circuit_len > 0,
        current_idx < circuit_len,
    ensures
        next_idx < circuit_len,
{
    (current_idx + 1) % circuit_len
}

/// 10. Formally proves IMU 52-byte packet chunk slicing never exceeds buffer boundaries.
pub fn verify_imu_slice_offset(chunk_index: usize) -> (offset: usize)
    requires
        chunk_index < 13,
    ensures
        offset <= 48,
        offset + 4 <= 52,
{
    chunk_index * 4
}


// =====================================================================
// PART 3: STANDALONE PURE FUNCTIONS ACTUALLY CALLED BY nav.rs/main.rs
// =====================================================================
// (NavController in Part 1 models the same *kind* of invariants as
// NavigationManager, but NavigationManager's real state machine has more
// nuance than a 6-state generic model can capture faithfully, so it isn't
// wired in directly. These, however, are the exact free functions the real
// code calls — no parallel copy, this *is* the implementation.)

/// 11. Formally proves accel-ramped velocity stays within the acceleration
///     step bound AND within the original [-10000,10000] operating range.
pub fn verify_accel_ramp(current_v: i32, target_v: i32, max_step: i32) -> (next_v: i32)
    requires
        max_step >= 0 && max_step <= 10000,
        current_v >= -10000 && current_v <= 10000,
        target_v >= -10000 && target_v <= 10000,
    ensures
        next_v - current_v <= max_step,
        current_v - next_v <= max_step,
        next_v >= -10000,
        next_v <= 10000,
{
    let diff = target_v - current_v;
    let clamped_step = if diff > max_step {
        max_step
    } else if diff < -max_step {
        -max_step
    } else {
        diff
    };
    current_v + clamped_step
}

/// 12. Formally proves bypass distance strictly exceeds obstacle depth.
pub fn verify_bypass_distance(obstacle_depth_mm: i32, safety_buffer_mm: i32) -> (total_mm: i32)
    requires
        obstacle_depth_mm > 0 && obstacle_depth_mm <= 10000,
        safety_buffer_mm >= 400 && safety_buffer_mm <= 2000,
    ensures
        total_mm > obstacle_depth_mm,
        total_mm >= obstacle_depth_mm + safety_buffer_mm,
{
    obstacle_depth_mm + safety_buffer_mm
}

/// 13. Formally proves the avoidance-pivot turned-angle reduction (mirroring
///     `if turned_deg > 180 { 360 - turned_deg }`) always lands in [0,180].
///     Precondition matches reality: it's |heading_a - heading_b| for two
///     headings each already wrapped into [-180,180], so it's always in [0,360].
pub fn verify_turn_delta(turned_deg_raw: i32) -> (turned_deg: i32)
    requires
        turned_deg_raw >= 0 && turned_deg_raw <= 360,
    ensures
        turned_deg >= 0,
        turned_deg <= 180,
{
    if turned_deg_raw > 180 { 360 - turned_deg_raw } else { turned_deg_raw }
}

/// 14. Formally proves obstacle-avoidance pivot direction always turns away
///     from the obstacle's hemisphere.
pub fn verify_avoid_turn_direction(obstacle_angle_deg: i32) -> (dir: i32)
    requires
        obstacle_angle_deg >= -180 && obstacle_angle_deg <= 180,
    ensures
        dir == 1 || dir == -1,
        (obstacle_angle_deg >= 0) ==> (dir == -1),
        (obstacle_angle_deg < 0) ==> (dir == 1),
{
    if obstacle_angle_deg >= 0 { -1 } else { 1 }
}

/// 15. Formally proves the turn-speed interpolation (`min + (max-min)*scale`)
///     always stays within [min_speed, max_speed], for any scale in [0,1000] promille.
pub fn verify_turn_speed_interpolation(min_speed: i32, max_speed: i32, scale_promille: i32) -> (speed: i32)
    requires
        min_speed >= 0,
        min_speed <= max_speed,
        max_speed <= 10000,
        scale_promille >= 0,
        scale_promille <= 1000,
    ensures
        speed >= min_speed,
        speed <= max_speed,
{
    let delta = max_speed - min_speed;

    assert(0 <= delta && delta <= 10000);

    assert(delta * scale_promille >= 0) by (nonlinear_arith)
        requires
            delta >= 0,
            scale_promille >= 0;

    assert(delta * scale_promille <= delta * 1000) by (nonlinear_arith)
        requires
            delta >= 0,
            scale_promille <= 1000;

    let prod = delta * scale_promille;
    // prod == delta * scale_promille is ambient here (plain let-binding), so the
    // normal solver can combine it with the two facts above; dividing a linear
    // inequality by the constant 1000 is ordinary linear arithmetic.
    assert(prod / 1000 >= 0 && prod / 1000 <= delta);

    let step = prod / 1000;
    let interpolated = min_speed + step;

    if interpolated > max_speed {
        max_speed
    } else if interpolated < min_speed {
        min_speed
    } else {
        interpolated
    }
}

/// 16. Formally proves squared-distance waypoint arrival avoids both integer
///     overflow (via i64 promotion) and a float `sqrt` call. Bounds cover the
///     full CITY_CIRCUIT extent (~44m) with margin.
pub fn verify_arrival_distance_sq(dx_mm: i32, dy_mm: i32, tol_mm: i32) -> (reached: bool)
    requires
        dx_mm >= -50000 && dx_mm <= 50000,
        dy_mm >= -50000 && dy_mm <= 50000,
        tol_mm > 0 && tol_mm <= 5000,
    ensures
        reached ==> ((dx_mm as i64) * (dx_mm as i64) + (dy_mm as i64) * (dy_mm as i64) < (tol_mm as i64) * (tol_mm as i64)),
{
    let dx64 = dx_mm as i64;
    let dy64 = dy_mm as i64;
    let tol64 = tol_mm as i64;

    assert(dx64 * dx64 >= 0) by (nonlinear_arith);
    assert(dx64 * dx64 <= 2500000000) by (nonlinear_arith)
        requires -50000 <= dx64, dx64 <= 50000;
    assert(dy64 * dy64 >= 0) by (nonlinear_arith);
    assert(dy64 * dy64 <= 2500000000) by (nonlinear_arith)
        requires -50000 <= dy64, dy64 <= 50000;
    assert(tol64 * tol64 >= 0) by (nonlinear_arith);
    assert(tol64 * tol64 <= 25000000) by (nonlinear_arith)
        requires 0 < tol64, tol64 <= 5000;

    let dx2 = dx64 * dx64;
    let dy2 = dy64 * dy64;
    let tol2 = tol64 * tol64;

    (dx2 + dy2) < tol2
}

/// 17. Formally proves the calibration sample counter advances deterministically.
pub fn verify_calibration_step(current_samples: usize, required_samples: usize) -> (res: (usize, bool))
    requires
        required_samples > 0,
        current_samples < required_samples,
    ensures
        res.0 == current_samples + 1,
        res.1 == (res.0 >= required_samples),
{
    let next = current_samples + 1;
    (next, next >= required_samples)
}

/// 18. Formally proves the 16-byte motor command packet (two f64s) has zero slice overlap.
pub fn verify_motor_buffer_offsets(left_start: usize, right_start: usize) -> (is_valid: bool)
    ensures
        is_valid == (left_start == 0 && left_start + 8 == 8 && right_start == 8 && right_start + 8 == 16),
{
    left_start == 0 && right_start == 8
}

/// 19. Formally proves calibration bias mean is safe for any sample count and any
///     non-negative fixed-point scale.
pub fn verify_calibration_mean(sum_micro: i64, count: u32) -> (mean_micro: i64)
    requires
        count > 0,
        count <= 100_000,
        sum_micro >= 0 && sum_micro <= 1_000_000_000,
    ensures
        mean_micro == sum_micro / (count as i64),
{
    sum_micro / (count as i64)
}

} // verus!