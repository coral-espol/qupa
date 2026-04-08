# QUPA — Installation Guide

Full setup for the robot (Raspberry Pi Zero W2, Ubuntu 24.04 Server) and the development PC (Ubuntu 24.04 / WSL2).

---

## Robot

### 1. OS

Ubuntu 24.04 Server (arm64) on Raspberry Pi Zero W2.  
Flash with Raspberry Pi Imager, enable SSH, set hostname `qupa`, user `qupa`.

#### Check dates and Add developer repository

Adding resource repository to list
```bash
echo "Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports/
Suites: noble-updates
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg" | sudo tee -a /etc/apt/sources.list.d/ubuntu.sources
```

Update and Upgrade
```bash
sudo apt update && sudo apt upgrade -y 
``` 

### 2. ROS2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y ros-jazzy-ros-base
```

### 3. System dependencies

```bash
sudo apt install -y python3-rpi-lgpio python3-opencv python3-pip
```

Enable I2C and SPI (required for floor sensor and LED strip):

```bash
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
```

### 4. Camera — libcamera RPi fork

Ubuntu 24.04 ships libcamera 0.2.0, which has a known bug (`prepareIsp()` assertion failure) with the OV5647 sensor (RPi Camera Module v1). The official Raspberry Pi fork fixes this.

See **[camera_setup.md](camera_setup.md)** for the full build instructions.

### 5. picamera2

```bash
sudo apt install -y libcap-dev
sudo pip install picamera2 --break-system-packages
```

Add the libcamera Python bindings to the Python path:

```bash
echo "/usr/local/lib/python3/dist-packages" | \
  sudo tee /usr/local/lib/python3.12/dist-packages/libcamera-new.pth
```

### 6. Floor colour sensor (TCS34725)

The sensor communicates over I2C (I2C bus 1 by default).

```bash
sudo pip install adafruit-circuitpython-tcs34725 --break-system-packages
```

Verify the sensor is wired correctly and detected:

```bash
i2cdetect -y 1
# Should show address 0x29
```

### 7. LED strip (APA102)

The strip communicates over SPI (hardware SPI0, CE0).

```bash
sudo pip install apa102-pi --break-system-packages
```

Verify SPI is available:

```bash
ls /dev/spidev0.*
# Should show /dev/spidev0.0
```

### 8. Workspace

```bash
mkdir -p ~/qupa_ws/src && cd ~/qupa_ws/src
git clone <repo-url> qupa
cd ~/qupa_ws
source /opt/ros/jazzy/setup.bash
colcon build
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/qupa_ws/install/setup.bash" >> ~/.bashrc
```

### 9. DDS environment

```bash
source ~/qupa_ws/src/qupa/ros2_env_robot.bash
```

Add to `~/.bashrc` to auto-source on login.

---

## PC / WSL2

### 1. ROS2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-image-transport-plugins
```

### 2. WSL2 mirrored networking

For DDS to reach the robot from WSL2, enable mirrored networking.  
Add to `C:\Users\<user>\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

Then from PowerShell:
```powershell
wsl --shutdown
```

Reopen WSL2. Verify with `ping 192.168.0.120`.

### 3. Workspace

```bash
mkdir -p ~/qupa_ws/src && cd ~/qupa_ws/src
git clone <repo-url> qupa
cd ~/qupa_ws
source /opt/ros/jazzy/setup.bash
colcon build
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/qupa_ws/install/setup.bash" >> ~/.bashrc
```

### 4. DDS environment

```bash
source ~/qupa_ws/src/qupa/ros2_env_pc.bash
```

Add to `~/.bashrc` to auto-source on login.
