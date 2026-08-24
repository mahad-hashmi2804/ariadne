import os
import socket
import struct
import time
import cv2
import numpy as np
import mujoco
import mujoco.viewer

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "sandbox_world.xml")

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 1. Actuator Receiver (16 bytes = 2 x f64 floats)
actuator_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
actuator_sock.bind(("127.0.0.1", 5555))
actuator_sock.setblocking(False)

# 2. Vision Senders
rgb_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
depth_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

renderer = mujoco.Renderer(model, height=240, width=320)
camera_id = model.camera("front_cam").id

# Reduced payload configuration
EXPECTED_BYTES = 16  # [v_left, v_right]
FORMAT_STRING = "2d"

fps_limit = 30.0
frame_interval = 1.0 / fps_limit
last_frame_time = time.time()

print("[Simulation] Server active.")
print(" -> Listening for 16-byte track velocity payloads on UDP 127.0.0.1:5555")
print(" -> Streaming RGB JPEG to UDP 127.0.0.1:5557")
print(" -> Streaming Depth PNG to UDP 127.0.0.1:5558")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        latest_bytes = None

        # Drain socket buffer to ensure lowest possible control latency
        while True:
            try:
                data_bytes, _ = actuator_sock.recvfrom(EXPECTED_BYTES)
                if len(data_bytes) == EXPECTED_BYTES:
                    latest_bytes = data_bytes
            except BlockingIOError:
                break

        # Map 2 track velocities across the 4 drive actuators
        if latest_bytes:
            v_left, v_right = struct.unpack(FORMAT_STRING, latest_bytes)
            # Left track motors
            data.ctrl[0] = v_left
            data.ctrl[1] = v_left
            # Right track motors
            data.ctrl[2] = v_right
            data.ctrl[3] = v_right

        mujoco.mj_step(model, data)
        viewer.sync()

        # Stream RGB & Depth Frames
        current_time = time.time()
        if current_time - last_frame_time >= frame_interval:
            # Color Stream
            renderer.disable_depth_rendering()
            renderer.update_scene(data, camera=camera_id)
            rgb_frame = renderer.render()
            bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            _, jpeg_buf = cv2.imencode('.jpg', bgr_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            rgb_sock.sendto(jpeg_buf.tobytes(), ("127.0.0.1", 5557))

            # Depth Stream
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera=camera_id)
            depth_frame = renderer.render()
            depth_mm = (np.clip(depth_frame, 0, 65.5) * 1000.0).astype(np.uint16)
            _, png_buf = cv2.imencode('.png', depth_mm)
            depth_sock.sendto(png_buf.tobytes(), ("127.0.0.1", 5558))

            last_frame_time = current_time