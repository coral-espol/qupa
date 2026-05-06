#!/usr/bin/env bash
# ── ROS2 Jazzy — Robot (192.168.0.120) ───────────────────────────────────────
# Usage: source ros2_env_robot.bash

source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=192.168.0.111

export FASTRTPS_DEFAULT_PROFILES_FILE=~/qupa_ws/src/qupa/fastdds_robot.xml

echo "[ROS2 ROBOT] Jazzy sourced — IP=${ROS_IP}, DOMAIN_ID=${ROS_DOMAIN_ID}"
