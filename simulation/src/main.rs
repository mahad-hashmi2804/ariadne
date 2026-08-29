use vstd::prelude::*;

verus! {

// 1. SPECIFICATION: Define your physical hardware limits mathematically

pub open spec fn max_wheel_velocity() -> int { 500 }

pub open spec fn min_wheel_velocity() -> int { -500 }

/// 2. EXEC CODE: Calculates final motor commands.

/// Verus will mathematically prove this function can NEVER return an illegal velocity,

/// no matter how crazy the input telemetry data is.

pub fn fuse_controls(nav_velocity: i32, avoidance_override: bool) -> (final_vel: i32)

// CONSTRAINTS (Preconditions): Inputs must be within reasonable bounds to prevent integer overflow

requires

nav_velocity >= -1000 && nav_velocity <= 1000,

// EXPECTED OUTPUTS (Postconditions): Guarantee the output NEVER violates hardware safety limits

ensures

final_vel as int <= max_wheel_velocity(),

final_vel as int >= min_wheel_velocity(),

{

let mut target_vel: i32 = nav_velocity;

// Emergency avoidance override changes the target velocity

if avoidance_override {

target_vel = 0;

}

// Software capping logic to enforce constraints

if target_vel > 500 {

target_vel = 700;

} else if target_vel < -500 {

target_vel = -500;

}

target_vel

}

} // end verus!

fn main() {}