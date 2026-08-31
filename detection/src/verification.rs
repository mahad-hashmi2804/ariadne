//! # Vision/Detection Formal Verification Module (Verus SMT Specifications)
//!
//! These are the *real* exec functions used by `vision.rs` and `main.rs` —
//! not a parallel spec-only copy. Verus checks their bodies against the
//! contracts below; the rest of the crate imports and calls them directly,
//! so there is exactly one implementation of this logic.

use vstd::prelude::*;

verus! {

// =====================================================================
// PART 1: REGION-OF-INTEREST BOUNDS
// =====================================================================

/// Horizontal/vertical scan-band boundaries, guaranteed to lie inside the image.
pub struct RoiBounds {
    pub x_start: u32,
    pub x_end: u32,
    pub y_start: u32,
    pub y_end: u32,
}

/// 1. Formally proves the ground-plane-filtering ROI never exceeds image bounds
///    — this is what makes the pixel-scan loop's indexing safe.
pub fn compute_roi_bounds(width: u32, height: u32) -> (b: RoiBounds)
    ensures
        b.x_start <= b.x_end,
        b.x_end <= width,
        b.y_start <= b.y_end,
        b.y_end <= height,
{
    let width64 = width as u64;
    let height64 = height as u64;

    let x_start64 = width64 * 20 / 100;
    let x_end64 = width64 * 80 / 100;
    let y_start64 = height64 * 12 / 100;
    let y_end64 = height64 * 42 / 100;

    // width*80 (and friends) can't overflow u64 for any u32 width, so these
    // multiplications above are already safe. What's left is showing the
    // constant-divisor arithmetic gives us the ordering/bound we need —
    // all linear, no nonlinear_arith required.
    assert(x_start64 <= x_end64);
    assert(x_end64 <= width64);
    assert(y_start64 <= y_end64);
    assert(y_end64 <= height64);

    // Each *_64 value is <= width64/height64 <= u32::MAX as u64, so these
    // casts back to u32 are lossless (Verus can see the bound above).
    RoiBounds {
        x_start: x_start64 as u32,
        x_end: x_end64 as u32,
        y_start: y_start64 as u32,
        y_end: y_end64 as u32,
    }
}

// =====================================================================
// PART 2: PIXEL ACCUMULATION (overflow & panic safety)
// =====================================================================

/// Comfortably above any realistic sensor resolution (a 16K x 16K frame is ~2.6e8 pixels).
pub open spec fn max_scanned_pixels() -> u64 { 1_000_000_000 }

/// 2. Formally proves a raw depth reading is only accepted within sensor bounds.
pub fn depth_in_range(depth_mm: u16, min_dist_mm: u16, max_dist_mm: u16) -> (in_range: bool)
    requires
        min_dist_mm <= max_dist_mm,
    ensures
        in_range == (depth_mm >= min_dist_mm && depth_mm <= max_dist_mm),
{
    depth_mm >= min_dist_mm && depth_mm <= max_dist_mm
}

/// Running totals for the centroid computation, carried across the pixel-scan loop.
pub struct PixelAccumulator {
    pub sum_depth_mm: u64,
    pub sum_x_pixels: u64,
    pub close_pixel_count: u64,
}

impl PixelAccumulator {
    /// 3. Formally proves a fresh accumulator starts at zero.
    pub fn new() -> (a: Self)
        ensures
            a.sum_depth_mm == 0,
            a.sum_x_pixels == 0,
            a.close_pixel_count == 0,
    {
        PixelAccumulator { sum_depth_mm: 0, sum_x_pixels: 0, close_pixel_count: 0 }
    }

    /// 4. Formally proves accumulating one matching pixel cannot overflow `u64`,
    ///    as long as the accumulator hasn't already seen an implausible number
    ///    of pixels. The bound is inductive: it holds after `new()`, and each
    ///    call re-establishes it, so any finite loop calling this is safe.
    pub fn accumulate(&mut self, depth_mm: u16, x: u32)
        requires
            old(self).close_pixel_count < max_scanned_pixels(),
            old(self).sum_depth_mm <= old(self).close_pixel_count * (u16::MAX as u64),
            old(self).sum_x_pixels <= old(self).close_pixel_count * (u32::MAX as u64),
        ensures
            final(self).close_pixel_count == old(self).close_pixel_count + 1,
            final(self).sum_depth_mm == old(self).sum_depth_mm + depth_mm as u64,
            final(self).sum_x_pixels == old(self).sum_x_pixels + x as u64,
            final(self).sum_depth_mm <= final(self).close_pixel_count * (u16::MAX as u64),
            final(self).sum_x_pixels <= final(self).close_pixel_count * (u32::MAX as u64),
    {
        self.sum_depth_mm = self.sum_depth_mm + depth_mm as u64;
        self.sum_x_pixels = self.sum_x_pixels + x as u64;
        self.close_pixel_count = self.close_pixel_count + 1;
    }
}

// =====================================================================
// PART 3: CENTROID & BEARING ANGLE (integer/fixed-point core)
// =====================================================================

/// 5. Formally proves the centroid average is computed without a division-by-zero
///    panic, given the caller's minimum-pixel-count gate.
pub fn compute_centroid(acc: &PixelAccumulator) -> (c: (u64, u64))
    requires
        acc.close_pixel_count > 0,
    ensures
        c.0 == acc.sum_depth_mm / acc.close_pixel_count,
        c.1 == acc.sum_x_pixels / acc.close_pixel_count,
{
    (acc.sum_depth_mm / acc.close_pixel_count, acc.sum_x_pixels / acc.close_pixel_count)
}

/// 6. Formally proves the bearing angle (in milli-degrees) is always clamped to the
///    sensor's declared +/-30 degree field of view, regardless of rounding at the
///    image edges. (The original float formula didn't actually guarantee this near
///    odd-width edges — the explicit clamp here is a real behavior improvement,
///    not just a proof convenience.)
pub fn compute_angle_milli_deg(centroid_x: u32, width: u32) -> (angle_milli_deg: i32)
    requires
        width > 0,
        centroid_x <= width,
    ensures
        angle_milli_deg >= -30000,
        angle_milli_deg <= 30000,
{
    let center_x = (width / 2) as i64;
    if center_x == 0 {
        return 0;
    }
    let diff = centroid_x as i64 - center_x;
    let scaled = (diff * 30000) / center_x;
    if scaled > 30000 {
        30000
    } else if scaled < -30000 {
        -30000
    } else {
        scaled as i32
    }
}

// =====================================================================
// PART 4: STREAM FALLBACK ESCALATION (clock abstracted to elapsed seconds)
// =====================================================================

pub struct FallbackDecision {
    /// True the moment the stream is transitioning from live to fallback.
    pub just_transitioned: bool,
    /// True if the extended-fallback critical warning should fire this cycle.
    pub emit_critical_warning: bool,
}

/// 7. Formally proves the critical warning only fires once the stream has been
///    down at least `extended_threshold_secs`, and then no more often than once
///    every `warning_repeat_interval_secs`. This is the exact decision logic
///    `StreamState` runs; it's called only after the caller has confirmed the
///    stream is stale, and it takes elapsed times as plain inputs rather than
///    reading `Instant::now()` itself — the clock read is the only unverified part.
pub fn decide_fallback(
    already_using_fallback: bool,
    fallback_elapsed_secs: u64,
    extended_threshold_secs: u64,
    has_previous_warning: bool,
    previous_warning_elapsed_secs: u64,
    warning_repeat_interval_secs: u64,
) -> (d: FallbackDecision)
    ensures
        d.just_transitioned == !already_using_fallback,
        d.emit_critical_warning ==> fallback_elapsed_secs >= extended_threshold_secs,
        (d.emit_critical_warning && has_previous_warning)
            ==> previous_warning_elapsed_secs >= warning_repeat_interval_secs,
{
    let just_transitioned = !already_using_fallback;
    let past_threshold = fallback_elapsed_secs >= extended_threshold_secs;
    let should_warn = past_threshold
        && (!has_previous_warning || previous_warning_elapsed_secs >= warning_repeat_interval_secs);
    FallbackDecision { just_transitioned, emit_critical_warning: should_warn }
}

} // verus!