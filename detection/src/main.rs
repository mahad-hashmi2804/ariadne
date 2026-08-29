//! # Detection Subsystem Entry Point
//!
//! Handles UDP frame ingestion for RGB and Depth sensor streams, manages independent
//! per-stream fallback lifecycles during sensor dropouts, performs sensor fusion,
//! and dispatches serialized obstacle telemetry to the `movement` crate.

mod vision;
mod verification;

use serde::Serialize;
use std::fs::File;
use std::io::Write;
use std::net::UdpSocket;
use std::panic::{self, AssertUnwindSafe};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use vision::{create_default_fallback_images, process_sensor_streams, ObstacleTelemetry};

// =============================================================================
// NETWORK INTERFACE CONFIGURATION
// =============================================================================

/// Local socket address for incoming RGB JPEG frames from the simulator.
const RGB_LISTEN_ADDR: &str = "127.0.0.1:5557";

/// Local socket address for incoming 16-bit Depth PNG frames from the simulator.
const DEPTH_LISTEN_ADDR: &str = "127.0.0.1:5558";

/// Target UDP endpoint for broadcasting serialized obstacle telemetry to `movement`.
const MOVEMENT_TARGET_ADDR: &str = "127.0.0.1:5556";

/// Maximum UDP datagram payload size (bytes) to handle full image buffers without truncation.
const MAX_DATAGRAM_SIZE: usize = 65535;

// =============================================================================
// TIMING AND STREAM LIFECYCLE CONSTANTS
// =============================================================================

/// Maximum duration allowed without receiving a live frame before marking a stream stale.
const STREAM_TIMEOUT: Duration = Duration::from_secs(3);

/// Time interval between saving paired live test frames to disk.
const CAPTURE_INTERVAL: Duration = Duration::from_secs(5);

/// Maximum number of live paired frame captures saved during a session.
const TOTAL_TEST_CAPTURES: u32 = 5;

/// Duration of continuous fallback operation required before escalating to critical warnings.
const EXTENDED_FALLBACK_THRESHOLD: Duration = Duration::from_secs(30);

/// Repeat interval for critical fallback warnings during prolonged sensor loss.
const EXTENDED_FALLBACK_REPEAT_INTERVAL: Duration = Duration::from_secs(10);

/// Target detection engine processing loop frequency in Hertz.
const LOOP_HZ: u64 = 30;

/// Microsecond cycle duration required to enforce the target processing rate.
const TARGET_CYCLE: Duration = Duration::from_micros(1_000_000 / LOOP_HZ);

/// Filename for the static RGB fallback image asset.
const FALLBACK_RGB_FILENAME: &str = "fallback_rgb.jpg";

/// Filename for the static Depth fallback image asset.
const FALLBACK_DEPTH_FILENAME: &str = "fallback_depth.png";

// =============================================================================
// TELEMETRY DOMAIN TYPES
// =============================================================================

/// Denotes the operational provenance of a processed frame buffer stream.
#[derive(Serialize, Clone, Copy, PartialEq, Eq, Debug)]
#[serde(rename_all = "lowercase")]
pub enum SourceKind {
    /// Telemetry derived from real-time live UDP sensor frames.
    Live,
    /// Telemetry derived from static offline fallback image buffers.
    Fallback,
}

/// Outgoing JSON payload wrapper incorporating obstacle telemetry with stream provenance.
#[derive(Serialize)]
struct TelemetryPayload<'a> {
    /// Embedded obstacle distance, bearing angle, and detection status.
    #[serde(flatten)]
    telemetry: &'a ObstacleTelemetry,
    /// Origin provenance of the RGB frame stream.
    rgb_source: SourceKind,
    /// Origin provenance of the Depth frame stream.
    depth_source: SourceKind,
}

// =============================================================================
// STREAM STATE MANAGEMENT
// =============================================================================

/// Tracks stream health, frame buffers, and fallback transitions for a single sensor channel.
struct StreamState {
    /// Diagnostic display label for logging (e.g., "RGB" or "Depth").
    label: &'static str,
    /// Latest raw frame buffer bytes (JPEG or PNG).
    data: Option<Vec<u8>>,
    /// Instant when the last live UDP packet was successfully received.
    last_live_at: Instant,
    /// Flag indicating whether the stream is currently operating on fallback data.
    using_fallback: bool,
    /// Instant when the stream transitioned into fallback mode, if active.
    fallback_since: Option<Instant>,
    /// Instant when the last extended fallback warning was logged.
    last_extended_warning: Option<Instant>,
}

impl StreamState {
    /// Initializes a stream state tracker, granting a grace period before fallback activation.
    fn new(label: &'static str) -> Self {
        Self {
            label,
            data: None,
            last_live_at: Instant::now(),
            using_fallback: false,
            fallback_since: None,
            last_extended_warning: None,
        }
    }

    /// Updates state upon receiving a live UDP byte payload, disengaging active fallbacks.
    fn mark_live(&mut self, bytes: Vec<u8>) {
        self.data = Some(bytes);
        self.last_live_at = Instant::now();
        if self.using_fallback {
            println!(
                "[Detection] Live {} stream resumed. Disengaging offline fallback.",
                self.label
            );
            self.using_fallback = false;
            self.fallback_since = None;
            self.last_extended_warning = None;
        }
    }

    /// Evaluates if stream timeout duration has elapsed without live data.
    fn is_stale(&self) -> bool {
        self.last_live_at.elapsed() >= STREAM_TIMEOUT
    }

    /// Engages offline fallback image data if the live stream is stale, escalating if sustained.
    fn engage_fallback_if_needed(&mut self, fallback_bytes: &Option<Vec<u8>>) {
        if !self.is_stale() {
            return;
        }

        let fallback_elapsed_secs = self
            .fallback_since
            .map(|since| since.elapsed().as_secs())
            .unwrap_or(0);
        let (has_previous_warning, previous_warning_elapsed_secs) = match self.last_extended_warning {
            Some(last) => (true, last.elapsed().as_secs()),
            None => (false, 0),
        };

        let decision = verification::decide_fallback(
            self.using_fallback,
            fallback_elapsed_secs,
            EXTENDED_FALLBACK_THRESHOLD.as_secs(),
            has_previous_warning,
            previous_warning_elapsed_secs,
            EXTENDED_FALLBACK_REPEAT_INTERVAL.as_secs(),
        );

        if decision.just_transitioned {
            println!(
                "[Detection] Live {} stream dropped. Engaging offline fallback...",
                self.label
            );
            self.using_fallback = true;
            self.fallback_since = Some(Instant::now());
        }

        if let Some(bytes) = fallback_bytes {
            self.data = Some(bytes.clone());
        } else {
            eprintln!(
                "[Detection] WARNING: no cached fallback data available for {}.",
                self.label
            );
        }

        if decision.emit_critical_warning {
            eprintln!(
                "[Detection] CRITICAL: {} stream has been running on static \
             fallback data for {}s. Telemetry derived from this stream \
             does not reflect real-world conditions. Check sensor connectivity.",
                self.label, fallback_elapsed_secs
            );
            self.last_extended_warning = Some(Instant::now());
        }
    }

    

    /// Returns the active data source classification.
    fn source(&self) -> SourceKind {
        if self.using_fallback {
            SourceKind::Fallback
        } else {
            SourceKind::Live
        }
    }
}

// =============================================================================
// FALLBACK ASSET CACHE
// =============================================================================

/// In-memory cache for disk-loaded fallback image assets to eliminate redundant I/O operations.
struct FallbackCache {
    rgb: Option<Vec<u8>>,
    depth: Option<Vec<u8>>,
}

impl FallbackCache {
    /// Loads fallback imagery from disk, generating default target assets if missing.
    fn load(base_dir: &Path) -> Self {
        let rgb_path = base_dir.join(FALLBACK_RGB_FILENAME);
        let depth_path = base_dir.join(FALLBACK_DEPTH_FILENAME);

        if !rgb_path.exists() || !depth_path.exists() {
            println!("[Detection] Static fallback files missing. Generating default targets...");
            if let Err(e) = create_default_fallback_images(base_dir) {
                eprintln!(
                    "[Detection] WARNING: failed to generate fallback images: {}. \
                     Fallback mode will be unavailable until this is resolved.",
                    e
                );
            }
        }

        let rgb = std::fs::read(&rgb_path)
            .map_err(|e| eprintln!("[Detection] WARNING: could not load {:?}: {}", rgb_path, e))
            .ok();
        let depth = std::fs::read(&depth_path)
            .map_err(|e| eprintln!("[Detection] WARNING: could not load {:?}: {}", depth_path, e))
            .ok();

        Self { rgb, depth }
    }

    /// Re-attempts loading missing assets if disk reads previously failed.
    fn refresh_if_missing(&mut self, base_dir: &Path) {
        if self.rgb.is_none() {
            self.rgb = std::fs::read(base_dir.join(FALLBACK_RGB_FILENAME)).ok();
        }
        if self.depth.is_none() {
            self.depth = std::fs::read(base_dir.join(FALLBACK_DEPTH_FILENAME)).ok();
        }
    }
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

/// Resolves the parent directory of the current executable for stable relative asset resolution.
fn resolve_base_dir() -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|p| p.to_path_buf()))
        .unwrap_or_else(|| PathBuf::from("."))
}

/// Drains a non-blocking UDP socket, returning the most recent payload if available.
fn drain_socket(socket: &UdpSocket, buf: &mut [u8]) -> Option<Vec<u8>> {
    match socket.recv_from(buf) {
        Ok((amt, _)) => Some(buf[..amt].to_vec()),
        Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => None,
        Err(e) => {
            eprintln!("[Detection] WARNING: socket recv error: {}", e);
            None
        }
    }
}

/// Attempts to persist live paired RGB/Depth image buffers to disk for offline inspection.
fn try_save_test_captures(
    base_dir: &Path,
    saved_count: &mut u32,
    last_saved_time: &mut Instant,
    rgb_data: &[u8],
    depth_data: &[u8],
    distance_m: f64,
) {
    let now = Instant::now();
    if *saved_count < TOTAL_TEST_CAPTURES && now.duration_since(*last_saved_time) >= CAPTURE_INTERVAL {
        *saved_count += 1;
        *last_saved_time = now;

        let rgb_path = base_dir.join(format!("frame_{}_rgb.jpg", saved_count));
        let depth_path = base_dir.join(format!("frame_{}_depth.png", saved_count));

        let write_operation = (|| -> std::io::Result<()> {
            File::create(&rgb_path)?.write_all(rgb_data)?;
            File::create(&depth_path)?.write_all(depth_data)?;
            Ok(())
        })();

        match write_operation {
            Ok(()) => println!(
                "[{}/{}] Saved paired frames: {:?} & {:?} | Distance: {:.2}m",
                saved_count, TOTAL_TEST_CAPTURES, rgb_path, depth_path, distance_m
            ),
            Err(e) => {
                eprintln!(
                    "[Detection] WARNING: failed to save test capture {}/{}: {}",
                    saved_count, TOTAL_TEST_CAPTURES, e
                );
                *saved_count -= 1;
            }
        }
    }
}

// =============================================================================
// MAIN ENTRY POINT
// =============================================================================

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rgb_socket = UdpSocket::bind(RGB_LISTEN_ADDR)?;
    let depth_socket = UdpSocket::bind(DEPTH_LISTEN_ADDR)?;
    rgb_socket.set_nonblocking(true)?;
    depth_socket.set_nonblocking(true)?;

    let telemetry_socket = UdpSocket::bind("127.0.0.1:0")?;
    let base_dir = resolve_base_dir();

    println!("[Detection] Engine online.");
    println!(" -> Listening for RGB frames on UDP {}", RGB_LISTEN_ADDR);
    println!(" -> Listening for Depth frames on UDP {}", DEPTH_LISTEN_ADDR);
    println!(" -> Broadcasting telemetry to UDP {}", MOVEMENT_TARGET_ADDR);
    println!(" -> Base directory: {:?}\n", base_dir);

    let running = Arc::new(AtomicBool::new(true));
    {
        let running_flag = Arc::clone(&running);
        if let Err(e) = ctrlc::set_handler(move || {
            println!("\n[Detection] Shutdown signal received. Exiting cleanly...");
            running_flag.store(false, Ordering::SeqCst);
        }) {
            eprintln!("[Detection] WARNING: could not register shutdown handler: {}", e);
        }
    }

    let mut rgb_buf = vec![0u8; MAX_DATAGRAM_SIZE];
    let mut depth_buf = vec![0u8; MAX_DATAGRAM_SIZE];
    let mut fallback_cache = FallbackCache::load(&base_dir);

    let mut rgb_state = StreamState::new("RGB");
    let mut depth_state = StreamState::new("Depth");

    let mut saved_count: u32 = 0;
    let mut last_saved_time = Instant::now() - CAPTURE_INTERVAL;

    while running.load(Ordering::SeqCst) {
        let loop_start = Instant::now();

        if let Some(bytes) = drain_socket(&rgb_socket, &mut rgb_buf) {
            rgb_state.mark_live(bytes);
        }
        if let Some(bytes) = drain_socket(&depth_socket, &mut depth_buf) {
            depth_state.mark_live(bytes);
        }

        if rgb_state.is_stale() || depth_state.is_stale() {
            fallback_cache.refresh_if_missing(&base_dir);
        }
        rgb_state.engage_fallback_if_needed(&fallback_cache.rgb);
        depth_state.engage_fallback_if_needed(&fallback_cache.depth);

        if let (Some(rgb_data), Some(depth_data)) = (&rgb_state.data, &depth_state.data) {
            let fusion_result = panic::catch_unwind(AssertUnwindSafe(|| {
                process_sensor_streams(rgb_data, depth_data)
            }));

            let fusion_outcome = match fusion_result {
                Ok(inner) => inner,
                Err(_) => {
                    eprintln!(
                        "[Detection] WARNING: sensor fusion panicked this cycle \
                         (likely an RGB/depth resolution mismatch). Skipping frame."
                    );
                    Err("sensor fusion panicked".into())
                }
            };

            match fusion_outcome {
                Ok(telemetry) => {
                    let payload = TelemetryPayload {
                        telemetry: &telemetry,
                        rgb_source: rgb_state.source(),
                        depth_source: depth_state.source(),
                    };

                    match serde_json::to_string(&payload) {
                        Ok(json_payload) => {
                            if let Err(e) = telemetry_socket.send_to(json_payload.as_bytes(), MOVEMENT_TARGET_ADDR) {
                                eprintln!("[Detection] WARNING: failed to send telemetry: {}", e);
                            }
                        }
                        Err(e) => {
                            eprintln!("[Detection] WARNING: failed to serialize telemetry: {}", e);
                        }
                    }

                    let fully_live = !rgb_state.using_fallback && !depth_state.using_fallback;
                    if fully_live {
                        try_save_test_captures(
                            &base_dir,
                            &mut saved_count,
                            &mut last_saved_time,
                            rgb_data,
                            depth_data,
                            telemetry.distance_m,
                        );
                    }
                }
                Err(e) => {
                    eprintln!("[Detection] WARNING: sensor fusion failed this cycle: {}", e);
                }
            }

            if !rgb_state.using_fallback {
                rgb_state.data = None;
            }
            if !depth_state.using_fallback {
                depth_state.data = None;
            }
        }

        let elapsed = loop_start.elapsed();
        if elapsed < TARGET_CYCLE {
            std::thread::sleep(TARGET_CYCLE - elapsed);
        }
    }

    println!("[Detection] Engine stopped.");
    Ok(())
}