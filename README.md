# QUPA — ROS2 Jazzy Robot

Differential-drive robot with IR proximity sensing and colour-target detection via a top-mounted mirror camera.

---

## Hardware

| Component | Model / Details |
|---|---|
| Compute | Raspberry Pi Zero W2 — Ubuntu 24.04 Server |
| Motors | 2× DC motors, GPIO PWM (BCM) |
| IR sensors | 6× GP2Y0E03 via TCA9548A I2C multiplexer |
| Camera | Raspberry Pi Camera (picamera2, 640×480) |
| PC / WSL2 | Ubuntu 24.04, ROS2 Jazzy — 192.168.0.111 |
| Robot IP | 192.168.0.120 |

---

## Prerequisites

### Robot (Raspberry Pi Zero W2)

```bash
sudo apt install ros-jazzy-ros-base python3-rpi-lgpio python3-opencv
pip install picamera2
```

### PC / WSL2

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-joint-state-publisher
```

**WSL2 only** — enable mirrored networking so DDS can reach the robot.
Add to `C:\Users\<user>\.wslconfig`:
```ini
[wsl2]
networkingMode=mirrored
```
Then run `wsl --shutdown` from PowerShell and reopen WSL.

---

## Workspace setup

```bash
mkdir -p ~/qupa_ws/src && cd ~/qupa_ws/src
git clone <repo-url> qupa
cd ~/qupa_ws
colcon build
source install/setup.bash
```

---

## Network / DDS

Both the robot and the PC must source their respective env files before running any ROS2 command. These set `ROS_DOMAIN_ID=0` and point DDS static peers to the other machine.

**On the robot:**
```bash
source ~/qupa_ws/src/qupa/ros2_env_robot.bash
```

**On the PC:**
```bash
source ~/qupa_ws/src/qupa/ros2_env_pc.bash
```

> If the robot's IP changes, update `ROS_STATIC_PEERS` in both files accordingly.

---

## Packages

| Package | Purpose |
|---|---|
| `qupa_hardware` | Hardware drivers — IR scanner, motor driver, camera |
| `qupa_description` | URDF/Xacro robot model for TF and visualisation |

---

## Launching

### Robot — full hardware stack

```bash
# IR sensors + motor driver
ros2 launch qupa_hardware hardware.launch.py

# Camera (normal operation)
ros2 launch qupa_hardware camera.launch.py

# Camera + calibration node (for RViz tuning)
ros2 launch qupa_hardware camera.launch.py calibration:=true
```

### PC — robot description (TF tree)

```bash
ros2 launch qupa_description description.launch.py
```

---

## Topics

| Topic | Type | Direction | Rate | Description |
|---|---|---|---|---|
| `/qupa_3A/scan` | `sensor_msgs/LaserScan` | robot → PC | 10 Hz | 8-slot scan (45° step, `base_link` frame) |
| `/qupa_3A/cmd_vel` | `geometry_msgs/Twist` | PC → robot | on demand | Linear (m/s) + angular (rad/s) command |
| `/qupa_3A/camera/image_filtered/compressed` | `sensor_msgs/CompressedImage` | robot → PC | 5 Hz | JPEG annotated frame (~30–50 KB) |
| `/qupa_3A/camera/image_calibration/compressed` | `sensor_msgs/CompressedImage` | robot → PC | 5 Hz | Calibration overlay (calibration mode only) |

### LaserScan slot layout

```
Slot  Angle   Direction   Channel
  0     0°       E          ch7
  1    45°       NE         ch6
  2    90°       N          ch5
  3   135°       NW         —  (inf)
  4   180°       W          ch2
  5   225°       —          ch3
  6   270°       S          ch1
  7   315°       —          (inf)
```

---

## Manual motor test

```bash
# Forward
ros2 topic pub --rate 10 /qupa_3A/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"

# Spin
ros2 topic pub --rate 10 /qupa_3A/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 1.0}}"

# Stop
ros2 topic pub --once /qupa_3A/cmd_vel geometry_msgs/msg/Twist "{}"
```

> The motor driver has a 0.3 s watchdog — motors stop automatically if no command is received.

---

## RViz visualisation

1. Launch the description on the PC: `ros2 launch qupa_description description.launch.py`
2. Open RViz2 and set **Fixed Frame** to `base_link`
3. Add displays:
   - **LaserScan** → `/qupa_3A/scan` (Style: Points, Size: 0.05 m)
   - **Image** → `/qupa_3A/camera/image_filtered/compressed`
   - **Image** → `/qupa_3A/camera/image_calibration/compressed` *(calibration mode only)*
   - **RobotModel** → to visualise the URDF

---

## Camera calibration (live tuning)

Launch with `calibration:=true`, then watch `/qupa_3A/camera/image_calibration/compressed` in RViz. Adjust parameters on the fly — the overlay updates immediately with no restart:

```bash
# Mask geometry
ros2 param set /qupa_3A/camera_calibration_node inner_radius_px 70
ros2 param set /qupa_3A/camera_calibration_node outer_radius_px 115
ros2 param set /qupa_3A/camera_calibration_node inner_offset_x -6

# HSV colour ranges
ros2 param set /qupa_3A/camera_calibration_node color_blue_lower "[100, 90, 70]"
ros2 param set /qupa_3A/camera_calibration_node color_blue_upper "[140, 255, 255]"

# Detection threshold
ros2 param set /qupa_3A/camera_calibration_node min_area 80
```

Once satisfied, copy the values into `qupa_hardware/config/camera.yaml` and rebuild.

---

## Configuration files

| File | Node | Key parameters |
|---|---|---|
| `config/ir_scanner.yaml` | `ir_scanner` | I2C addresses, sampling, loop rate (10 Hz), quadratic calibration coefficients |
| `config/motor.yaml` | `motor_node` | GPIO pins, wheel base (0.086 m), v\_max (0.08 m/s), PWM range, watchdog timeout |
| `config/camera.yaml` | `camera_node`, `camera_calibration_node` | Resolution, ring mask, pole exclusions, HSV ranges, JPEG quality |

---

## IR sensor calibration

Each channel has a quadratic calibration polynomial `dist = a·x² + b·x + c` defined in `ir_scanner.yaml`. To recalibrate a channel, measure known distances, fit new coefficients, then update:

```yaml
ir_scanner:
  ros__parameters:
    cal_ch1: [a, b, c]   # S
    cal_ch2: [a, b, c]   # W
    ...
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `RPi.GPIO not found` | Wrong GPIO library for Ubuntu | `sudo apt install python3-rpi-lgpio` |
| `No module named 'cv2'` | OpenCV missing | `sudo apt install python3-opencv` |
| Topics not visible across machines | DDS discovery | Source env files on both sides; check `ROS_STATIC_PEERS` IPs |
| Launch file not found after build | File not synced before build | `git pull` on robot, then `colcon build` |
| Motors stop immediately | Watchdog timeout | Publish `cmd_vel` at ≥ 4 Hz (timeout = 0.3 s) |
| LaserScan shows no points in RViz | Fixed Frame wrong | Set Fixed Frame to `base_link` |
