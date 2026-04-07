# Camera Setup — Raspberry Pi Camera (OV5647) on Ubuntu 24.04

## Background

Ubuntu 24.04 ships **libcamera 0.2.0** (upstream). This version has a known bug with the OV5647 sensor (Raspberry Pi Camera Module v1):

```
FATAL default ipa_base.cpp:396 assertion "it != buffers_.end()" failed in prepareIsp()
```

The camera is detected correctly but crashes immediately when starting the stream. The fix is to build the **Raspberry Pi fork of libcamera** (v0.7+), which contains OV5647-specific patches not present in the upstream release.

This is a known limitation of Ubuntu 24.04 on Raspberry Pi hardware — it affects all users of Ubuntu 24.04 + RPi Camera Module v1, regardless of whether they use ROS2 or not.

---

## Build libcamera RPi fork

This takes approximately 2–3 hours on a Pi Zero W2. Run it overnight or from a `screen`/`tmux` session.

### 1. Install build dependencies

```bash
sudo apt install -y \
  git meson ninja-build pkg-config cmake \
  python3-yaml python3-ply python3-jinja2 \
  libgnutls28-dev libssl-dev libudev-dev \
  libdw-dev libyaml-dev libcap-dev
```

### 2. Clone the RPi fork

```bash
git clone https://github.com/raspberrypi/libcamera.git ~/libcamera
cd ~/libcamera
```

### 3. Configure (vc4 pipeline only — Pi Zero W2)

```bash
meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4 \
  -Dipas=rpi/vc4 \
  -Dv4l2=true \
  -Dpycamera=enabled \
  -Dgstreamer=disabled \
  -Dtest=false \
  -Dcam=disabled \
  -Dqcam=disabled \
  -Ddocumentation=disabled
```

### 4. Build

Use `-j2` to limit parallelism and avoid OOM on the Pi Zero W2:

```bash
ninja -C build -j2
```

### 5. Install

```bash
sudo ninja -C build install
sudo ldconfig
```

### 6. Set library priority

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

### 7. Add Python bindings to path

```bash
echo "/usr/local/lib/python3/dist-packages" | \
  sudo tee /usr/local/lib/python3.12/dist-packages/libcamera-new.pth
```

Verify:

```bash
python3 -c "from libcamera import _libcamera; print(_libcamera.__file__)"
# Should print: /usr/local/lib/python3/dist-packages/libcamera/_libcamera...
```

---

## Verify camera works

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

A non-zero mean value (e.g. `mean: 120.5`) confirms the camera is working correctly.

---

## Notes

- The system libcamera (0.2.0) remains installed at `/lib/aarch64-linux-gnu/` — it is not removed, only deprioritized via `ld.so.conf.d`.
- The Raspberry Pi OS (Debian Bookworm) apt repository **cannot** be mixed with Ubuntu Noble packages due to dependency conflicts. Do not add it.
- If the robot OS is reinstalled, this entire process must be repeated.
