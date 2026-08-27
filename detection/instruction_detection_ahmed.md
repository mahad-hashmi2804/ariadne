# Detection Subsystem Guide

This document explains the functionality, execution, and testing procedures for the `detection` package.

## Overview

The `detection` package is a Rust-based microservice responsible for:
1.  **Ingesting** live RGB and Depth sensor streams via UDP.
2.  **Processing** frames to detect red targets using color segmentation.
3.  **Calculating** spatial telemetry (distance and bearing) for detected obstacles.
4.  **Broadcasting** results as JSON over UDP to the Movement subsystem.

## Architecture

-   `src/main.rs`: Orchestrates the UDP loop, handles frame synchronization, regulates execution rate, and manages offline fallback.
-   `src/vision.rs`: Contains the core computer vision logic for image decoding, red target detection (centroid calculation), metric derivation, and offline file generation.

## Features

### 1. 30 Hz Timing Regulation
To prevent CPU thrashing and align with the physical sensor output rates, the main ingestion loop in `main.rs` is throttled to exactly **30 Hz** (33.33ms per cycle). It uses a precise timer (`Instant::now()` and `elapsed()`) to sleep only for the remaining time of the cycle.

### 2. Automatic Offline Fallback
If the live UDP simulation feed drops (no packets received on Ports 5557 or 5558 for more than 3 seconds), the service automatically engages **Offline Fallback**:
-   It checks for `fallback_rgb.jpg` and `fallback_depth.png` in the directory.
-   If they do not exist, it automatically **auto-generates** these fallback images (featuring a perfect red target at a distance of 1.5 meters) so the program runs out of the box.
-   It loads and processes these static frames sequentially at 30 Hz, maintaining a continuous telemetry broadcast stream.
-   When live UDP packets resume, it seamlessly disengages fallback and switches back to the live streams.

### 3. Spatial Centroid & Depth Calculations
-   **Centroid Math**: Average $(X, Y)$ coordinate of all segmented red pixels (color filter bounds: R > 140, G < 80, B < 80).
-   **Depth Estimation**: True metric distance sampled directly from the 16-bit depth buffer at the object centroid, converted from millimeters to meters.
-   **Bearing / Angular Offset**: Computes the horizontal bearing angle relative to the center of the camera FOV, mapping pixel offset directly to degrees (from -30° to +30°).

---

## How to Run

### Prerequisite
Make sure you have Rust/Cargo installed.

### Executing the Service
From the root directory of the workspace, run:
```bash
cargo run --package detection
```

You will see:
```text
[Detection] Engine online.
 -> Listening for RGB frames on UDP 127.0.0.1:5557
 -> Listening for Depth frames on UDP 127.0.0.1:5558
 -> Broadcasting telemetry to UDP 127.0.0.1:5556

[Detection] Live UDP stream dropped. Engaging offline fallback...
[Detection] Static fallback files missing. Generating default targets...
```

---

## Testing

### 1. Automated Unit Tests
The subsystem includes rigorous programmatic tests to verify segmentation, centroid math, bearing logic, and depth map sampling accuracy.

Run the unit test suite:
```bash
cargo test --package detection
```

Successful output:
```text
running 2 tests
test vision::tests::test_no_target_detection ... ok
test vision::tests::test_red_target_detection ... ok

test result: ok. 2 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out
```

### 2. Manual Verification (Saving Frames)
When a live UDP stream is active, the engine saves 5 paired test frame files spaced 5 seconds apart (`frame_1_rgb.jpg` and `frame_1_depth.png` to `frame_5_rgb.jpg` and `frame_5_depth.png`) for debugging and visual analysis.

---

## PR Status & Verification
The branch is fully ready for a pull request to `main`:
1. **Merge Conflict Resolution:** Completed. The merge conflict in `.gitignore` was resolved cleanly.
2. **Type-Safety & Warnings:** Fixed type mismatches on the branch inside `vision.rs` and added the missing `ctrlc` dependency to `Cargo.toml`. Fixed clippy warnings in `main.rs`.
3. **Compilation:** The workspace compiles 100% warning-free (note: mujoco-rs requires MUJOCO_DOWNLOAD_DIR env var for full builds).
4. **Test Suite:** Both negative and positive target tests pass perfectly under automated validation.
5. **System Integration:** Telemetry format (JSON) is aligned with the `movement` subsystem listener.
