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
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
# Static DDS peer list — add or remove IPs as robots/machines join the network.
# Each entry is a machine that DDS will try to contact directly, bypassing
# multicast discovery (required for WSL2 and cross-subnet setups).
#
#   192.168.0.120  — qupa_3A  (Raspberry Pi Zero W2, robot 1)
#   192.168.0.105  — qupa_3B  (Raspberry Pi Zero W2, robot 2)
#   192.168.0.122  — qupa_3C  (Raspberry Pi Zero W2, robot 3)
export ROS_STATIC_PEERS="192.168.0.102;192.168.0.120;192.168.0.105;192.168.0.122;192.168.0.114"

export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fastdds_pc.xml"

echo "[ROS2 PC/WSL2] Jazzy sourced — IP=${ROS_IP}, DOMAIN_ID=${ROS_DOMAIN_ID}"
