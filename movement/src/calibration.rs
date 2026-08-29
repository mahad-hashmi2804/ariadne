//! # System Calibrator
//!
//! Accumulates initial IMU samples, computes Z-axis gyroscope bias, and synchronizes
//! spawn position before activating autonomous navigation routines.

use std::f64::consts::PI;
use crate::types::{ImuFrame, RobotPose};

pub struct SystemCalibrator {
    samples_required: usize,
    collected_samples: Vec<ImuFrame>,
    pub gyro_bias_z: f64,
    pub is_calibrated: bool,
}

impl SystemCalibrator {
    /// Constructs a calibrator expecting `samples` IMU data frames prior to activation.
    pub fn new(samples: usize) -> Self {
        Self {
            samples_required: samples,
            collected_samples: Vec::with_capacity(samples),
            gyro_bias_z: 0.0,
            is_calibrated: false,
        }
    }

    /// Appends an IMU frame sample, computing bias and setting initial spawn pose once capacity is met.
    pub fn add_sample(&mut self, frame: ImuFrame, robot: &mut RobotPose) -> bool {
        if self.is_calibrated {
            return true;
        }

        self.collected_samples.push(frame);
        print!(
            "\r[CALIBRATING] Sampling IMU & Pose... ({}/{})",
            self.collected_samples.len(),
            self.samples_required
        );

        if self.collected_samples.len() >= self.samples_required {
            let sum_gz: f64 = self.collected_samples.iter().map(|f| f.gyro[2] as f64).sum();
            self.gyro_bias_z = sum_gz / (self.collected_samples.len() as f64);

            let last_frame = self.collected_samples.last().unwrap();
            robot.position.x = last_frame.pos_x as f64;
            robot.position.y = last_frame.pos_y as f64;

            let mut init_heading = (last_frame.yaw_rad as f64) * (180.0 / PI);
            while init_heading > 180.0 { init_heading -= 360.0; }
            while init_heading < -180.0 { init_heading += 360.0; }
            robot.heading = init_heading;

            self.is_calibrated = true;
            println!("\n[CALIBRATION COMPLETE]");
            println!(" -> Gyro Z Bias: {:.6} rad/s", self.gyro_bias_z);
            println!(
                " -> Spawn Pose Synced: ({:.2}, {:.2}) @ {:.1}°\n",
                robot.position.x, robot.position.y, robot.heading
            );
        }

        self.is_calibrated
    }
}