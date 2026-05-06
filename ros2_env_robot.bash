#!/usr/bin/env bash
# ── ROS2 Jazzy — Robot (Pi Zero W2) ─────────────────────────────────────────
# Usage: source ros2_env_robot.bash

source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "[ROS2 ROBOT] Jazzy sourced — DOMAIN_ID=${ROS_DOMAIN_ID} | RMW=CycloneDDS"
