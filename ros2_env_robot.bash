#!/usr/bin/env bash
# ── ROS2 Jazzy — Robot (192.168.0.120) ───────────────────────────────────────
# Usage: source ros2_env_robot.bash

source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Tell DDS to use the physical network interface.
# Run `ip route | grep default` to confirm the interface name (eth0 / wlan0).
export ROS_IP=192.168.0.120

echo "[ROS2 ROBOT] Jazzy sourced — IP=${ROS_IP}, DOMAIN_ID=${ROS_DOMAIN_ID}"
