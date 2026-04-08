# QUPA — ROS2 Jazzy Robot

Differential-drive robot with IR proximity sensing and colour-target detection via a top-mounted mirror camera.

---

## Hardware

| Component | Model / Details |
|---|---|
| Compute | Raspberry Pi Zero W2 — Ubuntu 24.04 Server |
| Motors | 2× DC motors, GPIO PWM (BCM) |
| IR sensors | 6× GP2Y0E03 via TCA9548A I2C multiplexer |
| Camera | Raspberry Pi Camera v1 — OV5647, picamera2, 640×480 |
| Floor sensor | TCS34725 — RGB colour, I2C bus 1 (addr 0x29) |
| LED strip | APA102 — 24 LEDs, hardware SPI0 |
| PC / WSL2 | Ubuntu 24.04, ROS2 Jazzy — 192.168.0.111 |
| Robot IP | 192.168.0.120 |

---

## Setup

Full installation instructions are in **[docs/installation.md](docs/installation.md)**.

Camera setup requires building the Raspberry Pi libcamera fork due to a known Ubuntu 24.04 bug with the OV5647 sensor — see **[docs/camera_setup.md](docs/camera_setup.md)**.

### Quick summary

```bash
# Robot
sudo apt install ros-jazzy-ros-base python3-rpi-lgpio python3-opencv
sudo pip install picamera2 adafruit-circuitpython-tcs34725 apa102-pi \
  --break-system-packages
# + build libcamera RPi fork — see docs/camera_setup.md

# PC / WSL2
sudo apt install ros-jazzy-desktop ros-jazzy-joint-state-publisher \
  ros-jazzy-image-transport-plugins

# Workspace (both)
mkdir -p ~/qupa_ws/src && cd ~/qupa_ws/src
git clone <repo-url> qupa
cd ~/qupa_ws && colcon build && source install/setup.bash
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

| Package | Machine | Purpose |
|---|---|---|
| `qupa_msgs` | both | Custom message definitions (`Detection`, `DetectionArray`) |
| `qupa_hardware` | robot | Hardware drivers — IR scanner, motor driver, camera, floor sensor, LEDs |
| `qupa_description` | both | URDF/Xacro robot model for TF and visualisation |
| `qupa_desktop` | PC | RViz launch and configuration for visualisation |

---

## Launching

### Robot — full hardware stack

```bash
# IR sensors + motor driver + floor sensor + LEDs (sequential startup)
ros2 launch qupa_hardware hardware.launch.py

# Camera — detection only, no image stream (normal operation)
ros2 launch qupa_hardware camera.launch.py

# Camera — detection + calibration image stream (tuning mode)
ros2 launch qupa_hardware camera_calibration.launch.py
```

The `namespace` argument selects the robot (default `qupa_3A`):

```bash
ros2 launch qupa_hardware camera.launch.py namespace:=qupa_3B
```

### PC — visualisation

```bash
ros2 launch qupa_desktop view.launch.py
```

---

## Topics

| Topic | Type | Direction | Rate | Description |
|---|---|---|---|---|
| `/qupa_3A/scan` | `sensor_msgs/LaserScan` | robot → PC | 10 Hz | 8-slot scan (45° step, `base_link` frame) |
| `/qupa_3A/cmd_vel` | `geometry_msgs/Twist` | PC → robot | on demand | Linear (m/s) + angular (rad/s) command |
| `/qupa_3A/camera/detections` | `qupa_msgs/DetectionArray` | robot → PC | 3 Hz | All colour blobs detected in the ring |
| `/qupa_3A/camera/image_calibration/compressed` | `sensor_msgs/CompressedImage` | robot → PC | 3 Hz | Annotated JPEG for RViz (calibration mode only) |
| `/qupa_3A/floor/color` | `std_msgs/String` | robot → PC | 5 Hz | JSON: `{"label": "CYAN", "hsv": [h, s, v]}` |
| `/qupa_3A/leds/command` | `std_msgs/String` | PC → robot | on demand | JSON LED command (see LED section) |

### DetectionArray message

`qupa_msgs/DetectionArray` carries a `std_msgs/Header` and a list of `Detection` targets. Every blob found in the ring is reported — multiple detections of the same colour are possible.

```
# qupa_msgs/Detection
string  color        # BLUE | GREEN | RED
float32 distance_px  # distance from mirror centre in pixels
float32 angle_deg    # angle in degrees (0 = front, clockwise positive)
int32   area         # blob area in pixels²
float32 cx           # centroid x in full image coordinates
float32 cy           # centroid y in full image coordinates
```

Example — echo live detections:

```bash
ros2 topic echo /qupa_3A/camera/detections
```

### Floor sensor message

`std_msgs/String` with a JSON payload:

```json
{"label": "CYAN", "hsv": [176.4, 0.82, 0.61]}
```

Possible labels: `CYAN`, `MAGENTA`, `YELLOW`, `UNKNOWN`.

### LED command message

`std_msgs/String` with a JSON payload. Supported modes:

```bash
# Set all LEDs to one colour
ros2 topic pub --once /qupa_3A/leds/command std_msgs/msg/String \
  '{"data": "{\"mode\": \"set_all\", \"rgb\": [0, 0, 255]}"}'

# Set a specific range of LEDs
ros2 topic pub --once /qupa_3A/leds/command std_msgs/msg/String \
  '{"data": "{\"mode\": \"set_segment\", \"rgb\": [0, 255, 0], \"from\": 0, \"to\": 7}"}'

# Clear (turn off all)
ros2 topic pub --once /qupa_3A/leds/command std_msgs/msg/String \
  '{"data": "{\"mode\": \"clear\"}"}'
```

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

```bash
# PC
ros2 launch qupa_desktop view.launch.py
```

Or manually:

1. Launch the description on the PC: `ros2 launch qupa_description description.launch.py`
2. Open RViz2 and set **Fixed Frame** to `base_link`
3. Add displays:
   - **LaserScan** → `/qupa_3A/scan` (Style: Points, Size: 0.05 m)
   - **Image** → `/qupa_3A/camera/image_calibration/compressed` *(calibration mode only)*
   - **RobotModel** → to visualise the URDF

---

## Camera calibration (live tuning)

`camera_node` and `camera_calibration_node` are **fully independent**: each opens its own camera instance. The calibration node publishes only the annotated JPEG; the camera node publishes only detections. They can run simultaneously without interfering.

Launch calibration mode, then watch `/qupa_3A/camera/image_calibration/compressed` in RViz. Adjust parameters on the fly — the overlay updates immediately with no restart:

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

The overlay draws: outer ring (yellow), inner circle (magenta), pole exclusion lines (orange), bounding boxes per blob (colour-coded), and an orientation arrow from the inner circle centre to each detected centroid with distance and angle.

Once satisfied, copy the values into `qupa_hardware/config/camera.yaml` and rebuild.

---

## Configuration files

| File | Node | Key parameters |
|---|---|---|
| `config/ir_scanner.yaml` | `ir_scanner` | I2C addresses, sampling, loop rate (10 Hz), quadratic calibration coefficients |
| `config/motor.yaml` | `motor_node` | GPIO pins, wheel base (0.086 m), v\_max (0.08 m/s), PWM range, watchdog timeout |
| `config/camera.yaml` | `camera_node`, `camera_calibration_node` | Resolution, ring mask, pole exclusions, HSV ranges, publish rate |
| `config/floor_sensor.yaml` | `floor_sensor_node` | Loop rate, integration time, gain, HSV ranges per colour label |
| `config/leds.yaml` | `led_node` | LED count, global brightness, named segment index ranges |

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
| `assertion "it != buffers_.end()"` | libcamera 0.2.0 bug with OV5647 | Build RPi libcamera fork — see [docs/camera_setup.md](docs/camera_setup.md) |
| Camera image black / hangs | picamera2 using system libcamera | Check `ldconfig -p \| grep libcamera` — v0.7 must have priority |
| Camera colours look wrong (blue/red swapped) | Incorrect colour space conversion | picamera2 RGB888 returns BGR data; use `COLOR_BGR2HSV` and `imencode` directly without swapping |
| `No module named 'adafruit_tcs34725'` | Floor sensor library not installed | `sudo pip install adafruit-circuitpython-tcs34725 --break-system-packages` |
| Floor sensor not detected | I2C disabled or sensor not wired | Run `i2cdetect -y 1` — should show `0x29`; enable I2C with `raspi-config` |
| `No module named 'apa102_pi'` | LED library not installed | `sudo pip install apa102-pi --break-system-packages` |
| LED strip not responding | SPI disabled or wiring | Run `ls /dev/spidev0.*`; enable SPI with `raspi-config` |
| Topics not visible across machines | DDS discovery | Source env files on both sides; check `ROS_STATIC_PEERS` IPs |
| Launch file not found after build | File not synced before build | `git pull` on robot, then `colcon build` |
| Motors stop immediately | Watchdog timeout | Publish `cmd_vel` at ≥ 4 Hz (timeout = 0.3 s) |
| LaserScan shows no points in RViz | Fixed Frame wrong | Set Fixed Frame to `base_link` |
