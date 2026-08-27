mod vision;

use serde::Serialize;
use std::fs::File;
use std::io::Write;
use std::net::UdpSocket;
use std::panic::{self, AssertUnwindSafe};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use vision::{create_default_fallback_images, process_sensor_streams, ObstacleTelemetry};

// ---------------------------------------------------------------------
// Configuration constants (previously scattered magic numbers)
// ---------------------------------------------------------------------
const RGB_LISTEN_ADDR: &str = "127.0.0.1:5557";
const DEPTH_LISTEN_ADDR: &str = "127.0.0.1:5558";
const MOVEMENT_TARGET_ADDR: &str = "127.0.0.1:5556";

const MAX_DATAGRAM_SIZE: usize = 65535;

const STREAM_TIMEOUT: Duration = Duration::from_secs(3);
const CAPTURE_INTERVAL: Duration = Duration::from_secs(5);
const TOTAL_TEST_CAPTURES: u32 = 5;

// If a stream has been running on fallback data this long, escalate from a
// one-time notice to a recurring loud warning so degraded operation can't go
// unnoticed indefinitely.
const EXTENDED_FALLBACK_THRESHOLD: Duration = Duration::from_secs(30);
const EXTENDED_FALLBACK_REPEAT_INTERVAL: Duration = Duration::from_secs(10);

const LOOP_HZ: u64 = 30;
const TARGET_CYCLE: Duration = Duration::from_micros(1_000_000 / LOOP_HZ);

const FALLBACK_RGB_FILENAME: &str = "fallback_rgb.jpg";
const FALLBACK_DEPTH_FILENAME: &str = "fallback_depth.png";

// ---------------------------------------------------------------------
// Telemetry source tagging
// ---------------------------------------------------------------------
#[derive(Serialize, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
enum SourceKind {
    Live,
    Fallback,
}

/// Wraps the vision module's Telemetry with per-stream provenance so
/// downstream consumers (e.g. Movement) know whether a given reading
/// was built from live sensor data or static fallback imagery.
#[derive(Serialize)]
struct TelemetryPayload<'a> {
    #[serde(flatten)]
    telemetry: &'a ObstacleTelemetry,
    rgb_source: SourceKind,
    depth_source: SourceKind,
}

// ---------------------------------------------------------------------
// Per-stream state: tracks the latest frame, when it last arrived live,
// and whether it is currently being served from fallback data.
// ---------------------------------------------------------------------
struct StreamState {
    label: &'static str,
    data: Option<Vec<u8>>,
    last_live_at: Instant,
    using_fallback: bool,
    fallback_since: Option<Instant>,
    last_extended_warning: Option<Instant>,
}

impl StreamState {
    fn new(label: &'static str) -> Self {
        Self {
            label,
            data: None,
            // Matches original behavior: give the live feed a full
            // STREAM_TIMEOUT grace period at startup before ever engaging
            // fallback, rather than assuming failure from tick zero.
            last_live_at: Instant::now(),
            using_fallback: false,
            fallback_since: None,
            last_extended_warning: None,
        }
    }

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

    fn is_stale(&self) -> bool {
        self.last_live_at.elapsed() >= STREAM_TIMEOUT
    }

    /// Switch this stream to fallback data if it has gone stale and isn't
    /// already on fallback. Independent per stream, so a dead depth
    /// camera no longer masks (or is masked by) a healthy RGB feed.
    fn engage_fallback_if_needed(&mut self, fallback_bytes: &Option<Vec<u8>>) {
        if !self.is_stale() {
            return;
        }
        if !self.using_fallback {
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

        // Escalate if this stream has been degraded for a long time, so
        // sustained partial-fallback operation can't go silently unnoticed.
        if let Some(since) = self.fallback_since {
            if since.elapsed() >= EXTENDED_FALLBACK_THRESHOLD {
                let should_warn = match self.last_extended_warning {
                    None => true,
                    Some(last) => last.elapsed() >= EXTENDED_FALLBACK_REPEAT_INTERVAL,
                };
                if should_warn {
                    eprintln!(
                        "[Detection] CRITICAL: {} stream has been running on static \
                         fallback data for {:.0}s. Telemetry derived from this stream \
                         does not reflect real-world conditions. Check sensor connectivity.",
                        self.label,
                        since.elapsed().as_secs_f64()
                    );
                    self.last_extended_warning = Some(Instant::now());
                }
            }
        }
    }

    fn source(&self) -> SourceKind {
        if self.using_fallback {
            SourceKind::Fallback
        } else {
            SourceKind::Live
        }
    }
}

// ---------------------------------------------------------------------
// Fallback asset cache: loaded once from disk, not re-read every cycle.
// ---------------------------------------------------------------------
struct FallbackCache {
    rgb: Option<Vec<u8>>,
    depth: Option<Vec<u8>>,
}

impl FallbackCache {
    /// NOTE: `vision::create_default_fallback_images()` hardcodes its output
    /// paths ("fallback_rgb.jpg" / "fallback_depth.png") relative to the
    /// process's current working directory, and we're not modifying
    /// vision.rs. So — unlike the frame captures below, which main.rs writes
    /// itself and can place next to the executable — fallback assets MUST be
    /// read from the current working directory too, or generation and
    /// loading will silently point at two different places.
    fn load() -> Self {
        let rgb_path = PathBuf::from(FALLBACK_RGB_FILENAME);
        let depth_path = PathBuf::from(FALLBACK_DEPTH_FILENAME);

        if !rgb_path.exists() || !depth_path.exists() {
            println!("[Detection] Static fallback files missing. Generating default targets...");
            if let Err(e) = create_default_fallback_images() {
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

    /// Re-attempt loading if a fallback asset was missing at startup.
    fn refresh_if_missing(&mut self) {
        if self.rgb.is_none() {
            self.rgb = std::fs::read(FALLBACK_RGB_FILENAME).ok();
        }
        if self.depth.is_none() {
            self.depth = std::fs::read(FALLBACK_DEPTH_FILENAME).ok();
        }
    }
}

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

/// Resolve a stable base directory (next to the executable) instead of
/// relying on the process's current working directory, which varies by
/// how/where the binary is launched.
fn resolve_base_dir() -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|p| p.to_path_buf()))
        .unwrap_or_else(|| PathBuf::from("."))
}

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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Startup failures (can't bind sockets at all) are genuinely fatal —
    // there is nothing useful the engine can do, so `?` is appropriate here.
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

    // Graceful shutdown handling.
    let running = Arc::new(AtomicBool::new(true));
    {
        let running = Arc::clone(&running);
        if let Err(e) = ctrlc::set_handler(move || {
            println!("\n[Detection] Shutdown signal received. Exiting cleanly...");
            running.store(false, Ordering::SeqCst);
        }) {
            eprintln!("[Detection] WARNING: could not register shutdown handler: {}", e);
        }
    }

    let mut rgb_buf = vec![0u8; MAX_DATAGRAM_SIZE];
    let mut depth_buf = vec![0u8; MAX_DATAGRAM_SIZE];

    let mut fallback_cache = FallbackCache::load();

    let mut rgb_state = StreamState::new("RGB");
    let mut depth_state = StreamState::new("Depth");

    let mut saved_count: u32 = 0;
    let mut last_saved_time = Instant::now() - CAPTURE_INTERVAL;

    while running.load(Ordering::SeqCst) {
        let loop_start = Instant::now();

        // --- Ingest ---
        if let Some(bytes) = drain_socket(&rgb_socket, &mut rgb_buf) {
            rgb_state.mark_live(bytes);
        }
        if let Some(bytes) = drain_socket(&depth_socket, &mut depth_buf) {
            depth_state.mark_live(bytes);
        }

        // --- Per-stream fallback engagement (independent, not coupled) ---
        if rgb_state.is_stale() || depth_state.is_stale() {
            // Only worth trying to (re)load missing fallback assets if we
            // actually need them right now.
            fallback_cache.refresh_if_missing();
        }
        rgb_state.engage_fallback_if_needed(&fallback_cache.rgb);
        depth_state.engage_fallback_if_needed(&fallback_cache.depth);

        // --- Process whenever both streams have *some* data (live or fallback) ---
        if let (Some(rgb_data), Some(depth_data)) = (&rgb_state.data, &depth_state.data) {
            // vision::process_sensor_streams indexes the depth image using
            // coordinates derived from the RGB image's dimensions. If the two
            // streams ever arrive at different resolutions, that indexing
            // panics rather than returning an Err. We can't change vision.rs,
            // so we contain the blast radius here: catch the unwind, log it,
            // and skip this cycle instead of taking down the whole engine.
            let fusion_result = panic::catch_unwind(AssertUnwindSafe(|| {
                process_sensor_streams(rgb_data, depth_data)
            }));

            let fusion_result = match fusion_result {
                Ok(inner) => inner,
                Err(_) => {
                    eprintln!(
                        "[Detection] WARNING: sensor fusion panicked this cycle \
                         (likely an RGB/depth resolution mismatch). Skipping frame."
                    );
                    // Synthesize an Err so the existing match arm below handles it uniformly.
                    Err("sensor fusion panicked".into())
                }
            };

            match fusion_result {
                Ok(telemetry) => {
                    let payload = TelemetryPayload {
                        telemetry: &telemetry,
                        rgb_source: rgb_state.source(),
                        depth_source: depth_state.source(),
                    };

                    match serde_json::to_string(&payload) {
                        Ok(json_payload) => {
                            if let Err(e) =
                                telemetry_socket.send_to(json_payload.as_bytes(), MOVEMENT_TARGET_ADDR)
                            {
                                eprintln!(
                                    "[Detection] WARNING: failed to send telemetry to Movement: {}",
                                    e
                                );
                            }
                        }
                        Err(e) => {
                            eprintln!("[Detection] WARNING: failed to serialize telemetry: {}", e);
                        }
                    }

                    // Only persist test captures when BOTH streams are genuinely
                    // live — never save a frame that mixes live and fallback data.
                    let fully_live = !rgb_state.using_fallback && !depth_state.using_fallback;
                    let now = Instant::now();
                    if fully_live
                        && saved_count < TOTAL_TEST_CAPTURES
                        && now.duration_since(last_saved_time) >= CAPTURE_INTERVAL
                    {
                        saved_count += 1;
                        last_saved_time = now;

                        let rgb_path = base_dir.join(format!("frame_{}_rgb.jpg", saved_count));
                        let depth_path = base_dir.join(format!("frame_{}_depth.png", saved_count));

                        let save_result = (|| -> std::io::Result<()> {
                            File::create(&rgb_path)?.write_all(rgb_data)?;
                            File::create(&depth_path)?.write_all(depth_data)?;
                            Ok(())
                        })();

                        match save_result {
                            Ok(()) => println!(
                                "[{}/{}] Saved paired frames: {:?} & {:?} | Distance: {:.2}m",
                                saved_count,
                                TOTAL_TEST_CAPTURES,
                                rgb_path,
                                depth_path,
                                telemetry.distance_m
                            ),
                            Err(e) => {
                                eprintln!(
                                    "[Detection] WARNING: failed to save test capture {}/{}: {}",
                                    saved_count, TOTAL_TEST_CAPTURES, e
                                );
                                // Don't burn a capture slot on a failed write.
                                saved_count -= 1;
                            }
                        }
                    }
                }
                Err(e) => {
                    eprintln!("[Detection] WARNING: sensor fusion failed this cycle: {}", e);
                }
            }

            // Flush buffers for streams that are live (so we wait for the
            // next real packet); fallback streams keep re-serving cached
            // bytes each cycle by design.
            if !rgb_state.using_fallback {
                rgb_state.data = None;
            }
            if !depth_state.using_fallback {
                depth_state.data = None;
            }
        }

        // --- Enforce loop rate ---
        let elapsed = loop_start.elapsed();
        if elapsed < TARGET_CYCLE {
            std::thread::sleep(TARGET_CYCLE - elapsed);
        }
    }

    println!("[Detection] Engine stopped.");
    Ok(())
}