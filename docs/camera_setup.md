# Camera Setup — Raspberry Pi Camera (OV5647) on Ubuntu 24.04 Server

## Background

Ubuntu 24.04 ships **libcamera 0.2.0** (upstream). This version has a known bug with the OV5647 sensor (Raspberry Pi Camera Module v1):

```
FATAL default ipa_base.cpp:396 assertion "it != buffers_.end()" failed in prepareIsp()
```

The camera is detected correctly but crashes immediately when starting the stream. The fix is to build the **Raspberry Pi fork of libcamera** and **rpicam-apps** from source.

This is a known limitation of Ubuntu 24.04 on Raspberry Pi hardware — it affects all users of Ubuntu 24.04 + RPi Camera Module v1, regardless of whether they use ROS2.

> **Time estimate:** building libcamera takes 2–3 hours on a Pi Zero W2 with `-j1`. Run from a `screen` or `tmux` session so an SSH disconnect does not abort the build.

---

## 1. Remove conflicting packages

```bash
sudo apt remove --purge rpicam-apps
sudo apt remove --purge libcamera-dev libcamera0
```

---

## 2. Install build dependencies

```bash
# Essential build tools
sudo apt install -y git python3-pip python3-jinja2 meson cmake ninja-build

# libcamera dependencies
sudo apt install -y \
  libboost-dev libgnutls28-dev openssl libtiff5-dev pybind11-dev \
  python3-yaml python3-ply libglib2.0-dev \
  libgstreamer-plugins-base1.0-dev

# rpicam-apps dependencies
sudo apt install -y \
  libboost-program-options-dev libdrm-dev libexif-dev \
  libepoxy-dev libjpeg-dev libtiff5-dev libpng-dev

# Hardware encoder support
sudo apt install -y v4l-utils
```

> No Qt or EGL packages are needed — this is a headless server installation.

---

## 3. Build libcamera

### Clone

```bash
git clone https://github.com/raspberrypi/libcamera.git ~/libcamera
cd ~/libcamera
```

### Configure

```bash
meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp \
  -Dipas=rpi/vc4,rpi/pisp \
  -Dv4l2=true \
  -Dgstreamer=enabled \
  -Dtest=false \
  -Dlc-compliance=disabled \
  -Dcam=disabled \
  -Dqcam=disabled \
  -Ddocumentation=disabled \
  -Dpycamera=enabled
```

### Create a swap file (prevents OOM during build)

The Pi Zero W2 has 512 MB RAM. A swap file is needed for the compilation:

```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### Build

Use `-j1` to avoid memory exhaustion on the Pi Zero W2:

```bash
ninja -C build -j1
```

### Install

```bash
sudo ninja -C build install
sudo ldconfig
```

### Set library priority

The new libcamera installs to `/usr/local/lib/aarch64-linux-gnu/`. Give it priority over the system version:

```bash
echo "/usr/local/lib/aarch64-linux-gnu" | \
  sudo tee /etc/ld.so.conf.d/libcamera-local.conf
sudo ldconfig
```

Verify the correct version is active:

```bash
ldconfig -p | grep libcamera
# Should show: libcamera.so.0.7 => /usr/local/lib/aarch64-linux-gnu/libcamera.so.0.7
```

### Add Python bindings to path

```bash
echo "/usr/local/lib/python3/dist-packages" | \
  sudo tee /usr/local/lib/python3.12/dist-packages/libcamera-new.pth
```

Verify:

```bash
python3 -c "from libcamera import _libcamera; print(_libcamera.__file__)"
# Should print: /usr/local/lib/python3/dist-packages/libcamera/_libcamera...
```

```bash
cd ~
```

---

## 4. Build rpicam-apps

### Clone

```bash
git clone https://github.com/raspberrypi/rpicam-apps.git ~/rpicam-apps
cd ~/rpicam-apps
```

### Configure (headless / Ubuntu Server)

```bash
meson setup build \
  -Denable_libav=disabled \
  -Denable_drm=enabled \
  -Denable_egl=disabled \
  -Denable_qt=disabled \
  -Denable_opencv=disabled \
  -Denable_tflite=disabled \
  -Denable_hailo=disabled
```

### Build and install

```bash
meson compile -C build
sudo meson install -C build
sudo ldconfig
```

```bash
cd ~
```

---

## 5. Enable the camera in firmware

```bash
sudo nano /boot/firmware/config.txt
```

Add the overlay for the OV5647 sensor (Raspberry Pi Camera Module v1):

```
dtoverlay=ov5647
```

Other sensors for reference (not used here):
```
# dtoverlay=imx708   # Camera Module 3
# dtoverlay=imx477   # HQ Camera
# dtoverlay=imx519   # 16MP Camera
```

Save with `Ctrl+X`, then `Y`, then `Enter`.

---

## 6. Reboot

```bash
sudo reboot
```

---

## 7. Verify camera with rpicam-apps

```bash
# Check version
rpicam-hello --version

# List detected cameras
rpicam-hello --list-cameras

# Test stream for 5 seconds (headless — no preview window)
rpicam-hello -t 5000 --nopreview

# Take a still image
rpicam-still -o test.jpg --nopreview

# Record 10 seconds of video
rpicam-vid -t 10000 -o test.h264 --codec yuv420 --nopreview
```

> `--codec yuv420` is required for video recording. Without it, rpicam-vid fails with an encoder error.

---

## 8. Install picamera2

```bash
sudo apt install -y libcap-dev
sudo pip install picamera2 --break-system-packages
```

Verify:

```bash
python3 -c "
from picamera2 import Picamera2
cam = Picamera2()
cam.start()
import time; time.sleep(2)
f = cam.capture_array()
print('OK — mean pixel value:', f.mean())
cam.stop()
"
```

A non-zero mean value (e.g. `mean: 120.5`) confirms the full stack is working.

---

## Colour pipeline

picamera2 with format `RGB888` returns frames in **BGR byte order** (OpenCV convention). The correct pipeline throughout is:

```
capture_array()  →  BGR frame
                 →  cv2.COLOR_BGR2HSV   (colour detection)
                 →  cv2.imencode('.jpg', frame)   (JPEG — expects BGR, correct output)
```

Do **not** convert to RGB before `imencode` or before HSV conversion — this swaps the red and blue channels in both detection results and the published image.

---

## Notes

- The system libcamera (0.2.0) is removed in step 1, not just deprioritized.
- The Raspberry Pi OS (Debian Bookworm) apt repository **cannot** be mixed with Ubuntu Noble packages due to dependency conflicts. Do not add it.
- The swap file created in step 3 can be removed after the build: `sudo swapoff /swapfile && sudo rm /swapfile`.
- If the robot OS is reinstalled, this entire process must be repeated.
