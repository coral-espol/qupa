#!/usr/bin/env bash
# ── ROS2 Jazzy — PC Linux ────────────────────────────────────────────────────
# Usage: source ros2_env_pc.bash

source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "[ROS2 PC] Jazzy sourced — DOMAIN_ID=${ROS_DOMAIN_ID} | RMW=CycloneDDS"
