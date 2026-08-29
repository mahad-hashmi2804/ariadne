"""Ariadne MuJoCo Physics & Vision Engine Gateway.

Provides 3D interactive visualization via GLFW, renders camera streams
(JPEG RGB & PNG 16-bit Depth), streams 100 Hz binary IMU frames over UDP, and
converts 2D viewport mouse clicks into 3D world-space target raycasts.
"""

import json
import math
import os
import socket
import struct
import time
import cv2
import glfw
import mujoco
import numpy as np

# =============================================================================
# INITIALIZATION & MODEL LOADING
# =============================================================================

if not glfw.init():
    raise RuntimeError("Failed to initialize GLFW library")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "world.xml")

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

window = glfw.create_window(1280, 720, "Ariadne MuJoCo Simulation Engine", None, None)
if not window:
    glfw.terminate()
    raise RuntimeError("Failed to create GLFW window.")

glfw.make_context_current(window)
glfw.swap_interval(1)

cam = mujoco.MjvCamera()
opt = mujoco.MjvOption()
scn = mujoco.MjvScene(model, maxgeom=10000)
con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

renderer = mujoco.Renderer(model, height=240, width=320)
camera_id = model.camera("front_cam").id

mujoco.mjv_defaultCamera(cam)
mujoco.mjv_defaultOption(opt)

cam.azimuth = 90.0
cam.elevation = -30.0
cam.distance = 5.0
cam.lookat = [0.0, 0.0, 0.0]

# =============================================================================
# NETWORK SOCKET CONFIGURATION
# =============================================================================

actuator_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
actuator_sock.bind(("127.0.0.1", 5555))
actuator_sock.setblocking(False)

rgb_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
depth_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

imu_target = ("127.0.0.1", 5559)
nav_target = ("127.0.0.1", 5560)

EXPECTED_ACTUATOR_BYTES = 16
fps_limit = 30.0
frame_interval = 1.0 / fps_limit
last_frame_time = time.time()

imu_interval = 1.0 / 100.0
last_imu_time = time.time()

button_left = False
button_right = False
last_x, last_y = 0.0, 0.0
press_x, press_y = 0.0, 0.0
press_time = 0.0

# Gain applied to motor speed commands to ensure stable control response
CTRL_GAIN = 15.0

# =============================================================================
# RAYCASTING AND USER INTERACTION
# =============================================================================

def raycast_ground_target(win_w: int, win_h: int, mouse_x: float, mouse_y: float):
    """Projects viewport mouse coordinates into 3D world space, intersecting ground plane Z=0."""
    lookat = np.array(cam.lookat, dtype=np.float64)
    dist = cam.distance
    elev = math.radians(cam.elevation)
    azim = math.radians(cam.azimuth)
    fovy = math.radians(model.vis.global_.fovy)

    cam_pos = lookat + np.array([
        dist * math.cos(elev) * math.sin(azim),
        -dist * math.cos(elev) * math.cos(azim),
        dist * math.sin(elev)
    ])

    forward = lookat - cam_pos
    forward_norm = forward / np.linalg.norm(forward)

    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward_norm, world_up)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    else:
        right = right / np.linalg.norm(right)

    up = np.cross(right, forward_norm)

    ndc_x = (2.0 * mouse_x / win_w) - 1.0
    ndc_y = 1.0 - (2.0 * mouse_y / win_h)
    aspect = win_w / win_h

    tan_half_fovy = math.tan(fovy / 2.0)
    ray_dir = forward_norm + (ndc_x * aspect * tan_half_fovy) * right + (ndc_y * tan_half_fovy) * up
    ray_dir = ray_dir / np.linalg.norm(ray_dir)

    if abs(ray_dir[2]) < 1e-6:
        return None

    t = -cam_pos[2] / ray_dir[2]
    if t <= 0:
        return None

    target_pt = cam_pos + t * ray_dir
    tx = float(target_pt[0])
    ty = float(target_pt[1])

    tx = max(-10.0, min(10.0, tx))
    ty = max(-10.0, min(10.0, ty))

    return tx, ty

def mouse_button_cb(win, button, action, mods):
    """Handles mouse click events for camera rotation and ground target selection."""
    global button_left, button_right, press_x, press_y, press_time, last_x, last_y

    if button == glfw.MOUSE_BUTTON_LEFT:
        button_left = (action == glfw.PRESS)
        if action == glfw.PRESS:
            press_x, press_y = glfw.get_cursor_pos(win)
            press_time = time.time()
        elif action == glfw.RELEASE:
            release_x, release_y = glfw.get_cursor_pos(win)
            drag_dist = math.hypot(release_x - press_x, release_y - press_y)
            duration = time.time() - press_time

            if (drag_dist < 5.0 and duration < 0.3) or (mods & glfw.MOD_SHIFT):
                win_w, win_h = glfw.get_window_size(win)
                target = raycast_ground_target(win_w, win_h, release_x, release_y)
                if target:
                    tx, ty = target
                    print(f"\n[VIEWER CLICK] Target Selected at World Pos: ({tx:.2f}, {ty:.2f})")
                    payload = json.dumps({"x": tx, "y": ty}).encode("utf-8")
                    target_sock.sendto(payload, nav_target)

    elif button == glfw.MOUSE_BUTTON_RIGHT:
        button_right = (action == glfw.PRESS)

    last_x, last_y = glfw.get_cursor_pos(win)

def mouse_move_cb(win, xpos, ypos):
    """Updates interactive camera orbit and pan parameters upon mouse dragging."""
    global last_x, last_y

    dx = xpos - last_x
    dy = ypos - last_y
    win_w, win_h = glfw.get_window_size(win)

    if button_left:
        mujoco.mjv_moveCamera(model, mujoco.mjtMouse.mjMOUSE_ROTATE_H, dx / win_w, dy / win_h, cam)
        mujoco.mjv_moveCamera(model, mujoco.mjtMouse.mjMOUSE_ROTATE_V, dx / win_w, dy / win_h, cam)
    elif button_right:
        mujoco.mjv_moveCamera(model, mujoco.mjtMouse.mjMOUSE_MOVE_H, dx / win_w, dy / win_h, cam)
        mujoco.mjv_moveCamera(model, mujoco.mjtMouse.mjMOUSE_MOVE_V, dx / win_w, dy / win_h, cam)

    last_x, last_y = xpos, ypos

def scroll_cb(win, xoffset, yoffset):
    """Handles view zooming on mouse scroll wheel events."""
    mujoco.mjv_moveCamera(model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, cam)

glfw.set_mouse_button_callback(window, mouse_button_cb)
glfw.set_cursor_pos_callback(window, mouse_move_cb)
glfw.set_scroll_callback(window, scroll_cb)

print("[Simulation] Viewer active.")
print(" -> Listening for actuator commands on UDP 127.0.0.1:5555")
print(" -> Streaming RGB JPEG to UDP 127.0.0.1:5557")
print(" -> Streaming Depth PNG to UDP 127.0.0.1:5558")
print(" -> Streaming 52-byte IMU payload to UDP 127.0.0.1:5559 (100 Hz)")
print(" -> Streaming Click Targets to UDP 127.0.0.1:5560\n")

# =============================================================================
# MAIN SIMULATION CONTROL LOOP
# =============================================================================

while not glfw.window_should_close(window):
    latest_bytes = None
    while True:
        try:
            data_bytes, _ = actuator_sock.recvfrom(EXPECTED_ACTUATOR_BYTES)
            if len(data_bytes) == EXPECTED_ACTUATOR_BYTES:
                latest_bytes = data_bytes
        except BlockingIOError:
            break

    if latest_bytes:
        v_left, v_right = struct.unpack("2d", latest_bytes)
        data.ctrl[0] = v_left * CTRL_GAIN
        data.ctrl[1] = v_left * CTRL_GAIN
        data.ctrl[2] = v_right * CTRL_GAIN
        data.ctrl[3] = v_right * CTRL_GAIN

    mujoco.mj_step(model, data)
    current_time = time.time()

    # Stream IMU & ground-truth pose payload at 100 Hz
    if current_time - last_imu_time >= imu_interval:
        sim_t = float(data.time)
        acc = data.sensor("accel").data.astype(np.float32)
        gyro = data.sensor("gyro").data.astype(np.float32)
        mag = data.sensor("mag").data.astype(np.float32)

        real_x = float(data.qpos[0])
        real_y = float(data.qpos[1])

        w, x, y, z = data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]
        real_yaw = float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

        imu_packet = struct.pack(
            "<13f",
            sim_t,
            acc[0], acc[1], acc[2],
            gyro[0], gyro[1], gyro[2],
            mag[0], mag[1], mag[2],
            real_x, real_y, real_yaw
        )
        imu_sock.sendto(imu_packet, imu_target)
        last_imu_time = current_time

    # Stream RGB and Depth vision camera frames at 30 FPS
    if current_time - last_frame_time >= frame_interval:
        renderer.disable_depth_rendering()
        renderer.update_scene(data, camera=camera_id)
        rgb_frame = renderer.render()
        bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
        _, jpeg_buf = cv2.imencode('.jpg', bgr_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        if len(jpeg_buf) < 64000:
            rgb_sock.sendto(jpeg_buf.tobytes(), ("127.0.0.1", 5557))

        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=camera_id)
        depth_frame = renderer.render()
        depth_mm = (np.clip(depth_frame, 0, 65.5) * 1000.0).astype(np.uint16)

        _, png_buf = cv2.imencode('.png', depth_mm, [int(cv2.IMWRITE_PNG_COMPRESSION), 9])
        if len(png_buf) < 64000:
            try:
                depth_sock.sendto(png_buf.tobytes(), ("127.0.0.1", 5558))
            except OSError:
                pass

        last_frame_time = current_time

    glfw.make_context_current(window)
    viewport = mujoco.MjrRect(0, 0, *glfw.get_framebuffer_size(window))
    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
    mujoco.mjr_render(viewport, scn, con)

    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()