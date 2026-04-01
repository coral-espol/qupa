#!/usr/bin/env bash
# ── ROS2 Jazzy — PC / WSL2 (192.168.0.111) ───────────────────────────────────
# Requires WSL2 mirrored networking (Windows 11):
#   Add to C:\Users\<user>\.wslconfig:
#     [wsl2]
#     networkingMode=mirrored
#   Then run: wsl --shutdown
#
# Usage: source ros2_env_pc.bash

source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# With mirrored networking, WSL2 shares the Windows host IP.
# Verify with: ip addr show eth0   (or hostname -I)
export ROS_IP=192.168.0.111

echo "[ROS2 PC/WSL2] Jazzy sourced — IP=${ROS_IP}, DOMAIN_ID=${ROS_DOMAIN_ID}"
