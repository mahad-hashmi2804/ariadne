//! # JSON Payload String Parsers
//!
//! Provides parsing routines for custom target click overrides and vision telemetry.

use std::time::Instant;
use crate::types::{ObstacleFrame, Point2D};

/// Parses incoming JSON target strings (`{"x": 2.5, "y": -1.0}`) into 2D coordinates.
pub fn parse_target_json(payload: &str) -> Option<Point2D> {
    let clean = payload.trim_matches(|c| c == '{' || c == '}' || c == ' ' || c == '\n' || c == '\r');
    let mut x = None;
    let mut y = None;

    for kv in clean.split(',') {
        let parts: Vec<&str> = kv.split(':').collect();
        if parts.len() == 2 {
            let key = parts[0].trim().trim_matches('"');
            let val: f64 = parts[1].trim().parse().ok()?;
            if key == "x" { x = Some(val); }
            if key == "y" { y = Some(val); }
        }
    }

    if let (Some(x), Some(y)) = (x, y) {
        Some(Point2D { x, y })
    } else {
        None
    }
}

/// Parses obstacle telemetry JSON strings broadcast by `detection`.
pub fn parse_obstacle_json(payload: &str) -> Option<ObstacleFrame> {
    let clean = payload.trim_matches(|c| c == '{' || c == '}' || c == ' ' || c == '\n' || c == '\r');
    let mut detected = false;
    let mut distance_m = 0.0;
    let mut angle_deg = 0.0;

    for kv in clean.split(',') {
        let parts: Vec<&str> = kv.split(':').collect();
        if parts.len() == 2 {
            let key = parts[0].trim().trim_matches('"');
            let val_str = parts[1].trim();
            if key == "obstacle_detected" {
                detected = val_str.parse::<bool>().unwrap_or(false);
            } else if key == "distance_m" {
                distance_m = val_str.parse::<f64>().unwrap_or(0.0);
            } else if key == "angle_deg" {
                angle_deg = val_str.parse::<f64>().unwrap_or(0.0);
            }
        }
    }

    Some(ObstacleFrame {
        detected,
        distance_m,
        angle_deg,
        last_seen: Some(Instant::now()),
    })
}