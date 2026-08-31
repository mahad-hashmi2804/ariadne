# Ariadne: Autonomous Urban Search & Rescue (USAR) System

### **Comprehensive Engineering Wiki & Technical Reference**

**Architectural Design & Integration Lead:** Mahad Hashmi\
**Quality Assurance & Verification Lead:** Abeera Ahsan

Ariadne is a modular, formally verified edge robotics platform engineered for autonomous urban navigation and search-and-rescue operations. The system architecture decouples real-time kinematic control, depth-based vision processing, and physics simulation into asynchronous microservices communicating via local UDP sockets. This design supports offloading heavy perception tasks to dedicated NPU silicon while reserving real-time movement calculations for isolated microcontrollers.

---

## 1. System Architecture & Network Topology
_(**Socket Architecture & UDP Listeners:** Ameena Haque, Rameen Yasir & Ahmed Asif)_

The software suite consists of three primary subsystems: **Detection** (Rust), **Movement** (Rust), and **Simulation** (Python/MuJoCo).

```
                                +-------------------+
                                |   User / Viewer   |
                                +---------+---------+
                                          |
                                          | Target Raycast (UDP 5560)
                                          v
+------------------+   RGB (5557)   +-----+-------------+  Telemetry (5556)  +------------------+
|                  +--------------->+                   +------------------->+                  |
|    Simulation    |  Depth (5558)  |     Detection     |                    |     Movement     |
| (Python/MuJoCo)  +----------------+      (Rust)       |                    |      (Rust)      |
|                  |                +-------------------+                    |                  |
|                  |                                                         |                  |
|                  |<--------------------------------------------------------+                  |
|                  |                Track Commands (UDP 5555)                |                  |
|                  |                                                         |                  |
|                  +-------------------------------------------------------->+                  |
+------------------+                   IMU Data (UDP 5559)                  +------------------+

```

### UDP Network Interface Protocol

| Port | Sender → Receiver | Payload Format | Functional Description |
| --- | --- | --- | --- |
| **5555** | Movement → Simulation | 16-byte raw buffer (`2d` le) | Motor track target velocities `[left_v, right_v]` |
| **5556** | Detection → Movement | JSON (`TelemetryPayload`) | Obstacle detection status, distance, and angle |
| **5557** | Simulation → Detection | JPEG Bytes (`< 64 KB`) | Camera RGB frame buffer at 30 FPS |
| **5558** | Simulation → Detection | 16-bit PNG Bytes (`< 64 KB`) | Camera depth buffer at 30 FPS |
| **5559** | Simulation → Movement | 52-byte raw buffer (`13f` le) | Timestamp, Accel, Gyro, Mag, Position, Yaw |
| **5560** | Simulation → Movement | JSON (`Point2D`) | User mouse-click ground raycast override coordinates |

---

## 2. Setup & Execution Guide

### Prerequisites & Dependencies

* **Rust Toolchain**: `rustc` / `cargo` (Edition 2021)
* **Python 3.10+** with required visual & physics simulation libraries:
```bash
pip install mujoco glfw numpy opencv-python

```


* *(Optional)* Environment override for custom MuJoCo asset directories:
```bash
export MUJOCO_DOWNLOAD_DIR=$HOME/.mujoco

```



### System Launch Sequence (3 Separate Terminals)
_(**Created by:** Ahmed Asif)_

1. **Terminal 1 — Physics & Vision Simulation Engine**
```bash
python simulation/live_viewer.py

```


2. **Terminal 2 — Vision Processing & Fallback Engine**
```bash
cargo run --package detection

```


3. **Terminal 3 — Autonomous Movement & Kinematic Controller**
```bash
cargo run --package movement

```



### Runtime Data Loop Trace

1. **Simulation** steps physics model at 100 Hz and streams IMU frames (`5559`) to **Movement**.
2. **Simulation** renders camera buffers at 30 Hz, streaming JPEG camera feed (`5557`) & PNG depth encoding (`5558`) frames to **Detection**.
3. **Detection** processes 16-bit depth frames and broadcasts obstacle telemetry (`5556`) to **Movement**.
4. **Movement** updates internal state, solves differential steering vectors, and sends motor commands (`5555`) to **Simulation**.
5. **User** can click anywhere in the 3D viewer to project ground targets (`5560`), overriding autonomous circuit routes.

---

## 3. Detection Module (`detection/`)
_(**Team lead:** Ahmed Asif)_

The Detection engine ingests simulated vision feeds, filters point clouds for valid obstacle geometry, and streams structured JSON payloads.

### Vision Pipeline (`vision.rs`)
_(**Primary Authors:** Arshian Aqdas & Hooria Mansoor)_

* **Dynamic Horizon Region of Interest (ROI):** To prevent ground plane clutter from triggering false positives below chassis height ($z = 0.085\text{m}$), vertical scanning is restricted to $12\% \to 42\%$ of image height, and horizontal scanning is restricted to $20\% \to 80\%$ of image width.
* **Spatial Range Filtering:** Valid physical obstacles must fall strictly within the depth range of **0.3m ($300\text{mm}$) to 2.0m ($2000\text{mm}$)**.
* **Centroid & Angular Offset:** If $>100$ valid depth pixels register inside the ROI, the module computes average distance and horizontal centroid offset, mapping the centroid to a bounded camera FOV ($\pm 30^\circ$).

### Fault-Tolerant Stream Lifecycle (`main.rs`)
_(**Primary Authors:** Fazzal Abbas & Haroon Sher Mirza)_


* **Per-Stream State Isolation:** `StreamState` tracks RGB and Depth streams independently. If a camera feed goes stale ($>3.0\text{s}$ without packets), that feed automatically engages static offline assets (`fallback_rgb.jpg` / `fallback_depth.png`).
* **Escalation Warnings:** Extended operation on fallback data ($>30\text{s}$) triggers periodic `CRITICAL` console warnings every 10 seconds.
* **Panic Isolation:** Image processing calls are wrapped in `panic::catch_unwind` to prevent resolution mismatches from crashing the vision service.

---

## 4. Movement Module (`movement/`)
_(**Team leads:** Abdul Moiz & Kainat Mansha)_

The Movement engine manages system calibration, sensor integration, state transitions, and differential motor output.

### System Calibration (`calibration.rs`)
_(**Primary Authors:** Hamza Zafar & Shoaib Muhammad)_

Upon startup, actuators remain locked while `SystemCalibrator` collects **50 IMU samples**. It computes the Z-axis gyroscope bias to null out rotational drift and syncs spawn coordinates before enabling navigation.

### Autonomous State Machine (`nav.rs`)
_(**State Management & Telemetry Authors:** Rameen Yasir & Ameena Haque | **Module Lead:** Abdul Moiz)_

* **`Idle`**: Robot stationary, waiting for goal assignments.
* **`Turning`**: Rotating in place toward goal bearing. Transitions to `Moving` when heading error $\le 2.5^\circ$.
* **`Moving`**: Driving forward using proportional differential steering ($K_p = 0.02$).
* **`AvoidingTurn`**: Triggered when an obstacle breaks the $1.5\text{m}$ critical threshold. Pivots away from obstacle centroid up to a maximum sweep of $45^\circ$.
* **`AvoidingBypass`**: Drives straight for a calculated distance ($d_{\text{obstacle}} + 0.5\text{m}$) to clear barrier geometry before recalculating a path to the goal.
* **`Reached`**: Triggered when distance to target is $<0.3\text{m}$, advancing to the next waypoint in `CITY_CIRCUIT`.

### Kinematic Ramping & Velocity Control
_(**Primary Authors:** Hafiz Muhammad Umais Amjad & Hassaan Shafqat)_

Continuous acceleration limits (`max_accel = 15.0`) clamp per-frame velocity adjustments, eliminating current spikes and physical tipping during hard maneuvers.

### Inverse Kinemtics & Differential Steering
_(**Primary Authors:** Momina Anwar, Sharmeen Abbas & Roshanay Abid | **Module Lead:** Kainat Mansha)_

Low level track velocity commands computed from high-level linear and angular velocity targets.

---

## 5. Simulation Environment (`simulation/`)

### Interactive MuJoCo Engine (`live_viewer.py`)
_(**Primary Authors:** Muhammad Abdullah)_

* **Actuation Bridge:** Accepts 16-byte UDP packets containing track velocity targets and applies them to joint velocity actuators via balanced control gains (`CTRL_GAIN = 15.0`).
* **Interactive Target Selector:** Converts 2D viewport mouse clicks into 3D world-space ground plane intersections via raycasting, sending `Point2D` target overrides over UDP (`5560`).
* **UDP Buffer Safety:** Depth frames are encoded using Level 9 PNG compression and capped at 64,000 bytes to prevent socket buffer overflows.

### Physical Assets

* **Tracked Chassis (`robot.xml`):** _(**Primary Authors:** Ali Tayyab & Mujtaba Ahmed Khan)_\
 Differential skid-steer robot featuring active drive wheels, idlers, rubber tread friction profiles, an IMU sensor mount, and a front camera mount.
* **Urban Environment (`world.xml`):** _(**Primary Authors:** Sana Batool & Hafsa Ehtisham)_\
City plaza scene containing paved avenues, crosswalks, elevated sidewalk curbs ($z = 0.02\text{m}$), building structures, and an industrial rubble field with angled ramps for mobility testing.

---

## 6. Formal Verification Suite (Verus / Z3 SMT)

Ariadne contains **25 SMT-backed formal proofs** across two crate-level `verification.rs` modules, mathematically guaranteeing that core navigation and vision algorithms operate with zero arithmetic overflows, buffer panics, division-by-zero errors, or illegal state transitions.

```
                   +-------------------------------------------------------+
                   |     Ariadne Formal Verification Suite (25/25)         |
                   +---------------------------+---------------------------+
                                               |
                     +-------------------------+-------------------------+
                     |                                                   |
                     v                                                   v
      +------------------------------+                    +------------------------------+
      |      Movement Invariants     |                    |     Detection Invariants     |
      |          (18 Proofs)         |                    |          (7 Proofs)          |
      +--------------+---------------+                    +--------------+---------------+
                     |                                                   |
                     +-- NavController Constructor State                 +-- Ground-Plane ROI Image Bounds
                     +-- Idle State Machine Guards                       +-- Sensor Range Bounds Validation
                     +-- Frame Velocity Step Bounds                      +-- Accumulator Zero Initialization
                     +-- Bypass Trajectory Overshoot                     +-- Inductive Loop Accumulate Bounds
                     +-- Heading Modulo Wrap [-180, 180]                 +-- Zero-Division Centroid Protection
                     +-- Depth Sensor Noise Rejection                    +-- FOV Angle Clamping [-30°, +30°]
                     +-- Track Differential Clamping                     +-- Fallback & Alert Lifecycle
                     +-- Waypoint Circuit Index Wrap
                     +-- 52-Byte IMU Chunk Slicing
                     +-- Executable Velocity Ramp Bounds
                     +-- Bypass Clearance Overshoot
                     +-- Avoidance Pivot Delta Clamping
                     +-- Hemisphere Pivot Selection
                     +-- Promille Turn Speed Scale
                     +-- 64-Bit Squared Distance Check
                     +-- Calibration Sample Step State
                     +-- 16-Byte Motor Packet Offset
                     +-- Fixed-Point Gyro Mean Safety
```

### Movement Invariants (`movement/src/verification.rs` — 18 Proofs)

**Part 1: Stateful Controller Model**

1. **`NavController::new`**: Formally proves the constructor initializes safe zero-velocity defaults and idle state.
2. **`NavController::set_state`**: Proves state transitions out of `Idle` are strictly restricted to valid target states (`Turning`, `Reached`).
3. **`NavController::ramp_velocities`**: Uses `old(self)` and `final(self)` state bounds to prove per-frame velocity mutations never exceed `max_accel_step`.
4. **`NavController::update_bypass_distance`**: Proves calculated bypass trajectories strictly overshoot obstacle depth plus the required safety buffer.
   **Part 2: Domain Verification Helpers**
5. **`verify_angle_normalize`**: Proves raw heading angle differences map strictly within [−180∘,180∘] modulo 360∘.
6. **`verify_depth_threshold`**: Proves raw sensor depth values outside valid operational thresholds are mathematically rejected.
7. **`verify_differential_steering`**: Proves differential track speed splits never exceed mechanical steering thresholds.
8. **`verify_circuit_index_advance`**: Proves waypoint array indexing (`(idx + 1) % len`) cannot exceed circuit array bounds.
9. **`verify_imu_slice_offset`**: Proves slicing 52-byte UDP telemetry datagrams into 4-byte chunks stays strictly within buffer bounds.

**Part 3: Executable Runtime Invariants**
10. **`verify_accel_ramp`**: Proves velocity step changes remain within acceleration bounds while enforcing the [−10000,10000] operating envelope.
11. **`verify_bypass_distance`**: Proves bypass path clearance strictly exceeds barrier depth.
12. **`verify_turn_delta`**: Proves turned-angle reduction math (`360 - turned_deg`) always lands within [0∘,180∘].
13. **`verify_avoid_turn_direction`**: Proves obstacle pivot direction choices strictly turn away from the detected obstacle hemisphere.
14. **`verify_turn_speed_interpolation`**: Proves turn-speed promille scaling stays bounded within [min_speed,max_speed].
15. **`verify_arrival_distance_sq`**: Proves squared Euclidean distance checks prevent floating-point `sqrt` operations and avoid 32-bit overflow via 64-bit promotion.
16. **`verify_calibration_step`**: Proves calibration sample counting advances deterministically to completion.
17. **`verify_motor_buffer_offsets`**: Proves writing 64-bit track velocities to a 16-byte UDP packet buffer contains zero slice overlap.
18. **`verify_calibration_mean`**: Proves fixed-point gyro bias mean calculations across arbitrary sample counts are protected against division-by-zero panics.

### Detection Invariants (`detection/src/verification.rs` — 7 Proofs)

1. **`compute_roi_bounds`**: Formally proves that constant-divisor scan-band arithmetic (20%→80% X, 12%→42% Y) stays strictly within u32 image dimensions without buffer panics.
2. **`depth_in_range`**: Proves raw pixel depth filtering strictly enforces bounds between `min_dist_mm` and `max_dist_mm`.
3. **`PixelAccumulator::new`**: Proves fresh pixel accumulation structs initialize depth, X-coordinate, and pixel counts at zero.
4. **`PixelAccumulator::accumulate`**: Inductively proves that adding valid depth pixels to running totals cannot overflow `u64` storage across loops up to 109 pixels.
5. **`compute_centroid`**: Proves spatial centroid depth and X-coordinate averaging cannot trigger division-by-zero panics given minimum-pixel gates.
6. **`compute_angle_milli_deg`**: Proves centroid horizontal bearing angles (milli-degrees) are clamped to the physical camera field of view (±30000 mdeg).
7. **`decide_fallback`**: Proves critical extended-fallback alerts fire if and only if a stream has been degraded for ≥30s and repeat intervals (≥10s) are satisfied.

### Unified Execution Architecture

Ariadne links `vstd` directly into the project runtime. The functions in Part 3 of `movement` and Parts 1–4 of `detection` are **`exec fn` blocks**, serving as the single source of truth for both SMT theorem proving and production binaries.

```
+-------------------------------------------------------------------------+
|                       Unified Verus / Rust AST                          |
+------------------------------------+------------------------------------+
                                     |
              +----------------------+----------------------+
              |                                             |
              v                                             v
  [verus.exe Verification]                      [cargo build Binary]
  • Evaluates contracts & lemmas               • Compiles `exec fn` blocks
  • Z3 proves postconditions                   • Direct execution on hardware
  • Rejects unsafe math                        • Zero spec/code divergence
```

---
