import os
import socket
import struct
import time
import cv2
import numpy as np
import mujoco
import mujoco.viewer

# 1. Locate and Load the Sandbox World Model
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "world.xml")

print(f"[Simulation] Loading model from: {MODEL_PATH}")
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 2. Network Sockets Configuration
# Actuator Command Listener (Receives 18 x f64 targets from Movement)
actuator_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
actuator_sock.bind(("127.0.0.1", 5555))
actuator_sock.setblocking(False)

# RGB Stream Sender (Port 5557)
rgb_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rgb_target = ("127.0.0.1", 5557)

# Depth Stream Sender (Port 5558)
depth_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
depth_target = ("127.0.0.1", 5558)

# 3. Offscreen Renderer Setup (320x240 Resolution)
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
renderer = mujoco.Renderer(model, height=FRAME_HEIGHT, width=FRAME_WIDTH)

try:
    camera_id = model.camera("front_cam").id
except KeyError:
    print("[Simulation] ERROR: 'front_cam' not found in robot.xml! Check camera tag.")
    exit(1)

# Buffer Format Expected on Port 5555 (18 doubles = 144 bytes)
expected_bytes = model.nu * 8
format_string = f"{model.nu}d"

# 30 FPS Timing Limit
fps_limit = 30.0
frame_interval = 1.0 / fps_limit
last_frame_time = time.time()

print("[Simulation] Server active.")
print(" -> Listening for actuator commands on UDP 127.0.0.1:5555")
print(" -> Streaming RGB JPEG feed to UDP 127.0.0.1:5557 (30 FPS)")
print(" -> Streaming Depth PNG feed to UDP 127.0.0.1:5558 (30 FPS)")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # A. Non-blocking drain of incoming actuator targets from Rust
        latest_actuator_bytes = None
        while True:
            try:
                data_bytes, _ = actuator_sock.recvfrom(expected_bytes)
                if len(data_bytes) == expected_bytes:
                    latest_actuator_bytes = data_bytes
            except BlockingIOError:
                break

        # Apply updated target positions to MuJoCo actuators
        if latest_actuator_bytes:
            ctrl_targets = struct.unpack(format_string, latest_actuator_bytes)
            for i in range(model.nu):
                data.ctrl[i] = ctrl_targets[i]

        # B. Step Physics Simulation & Sync Interactive Viewer
        mujoco.mj_step(model, data)
        viewer.sync()

        # C. Render and Stream Camera Telemetry at 30 FPS
        current_time = time.time()
        if current_time - last_frame_time >= frame_interval:
            # 1. Render RGB Color Frame
            renderer.disable_depth_rendering()
            renderer.update_scene(data, camera=camera_id)
            rgb_frame = renderer.render()

            # Convert RGB to BGR for OpenCV JPEG encoding
            bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            _, jpeg_buffer = cv2.imencode('.jpg', bgr_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            jpeg_bytes = jpeg_buffer.tobytes()

            if len(jpeg_bytes) < 65507:
                rgb_sock.sendto(jpeg_bytes, rgb_target)

            # 2. Render Depth Frame (Metric float array)
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera=camera_id)
            depth_frame_meters = renderer.render()

            # Convert meters to uint16 millimeters (0.0m - 65.5m -> 0 - 65535 mm)
            depth_mm = (np.clip(depth_frame_meters, 0, 65.5) * 1000.0).astype(np.uint16)

            # Compress to 16-bit single channel PNG (~10-15KB)
            _, png_buffer = cv2.imencode('.png', depth_mm)
            png_bytes = png_buffer.tobytes()

            if len(png_bytes) < 65507:
                depth_sock.sendto(png_bytes, depth_target)

            last_frame_time = current_time