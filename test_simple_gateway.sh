#!/usr/bin/env bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source $(dirname $0)/install/setup.bash

# Set environment
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "Starting simple CAN gateway node..."
exec $(dirname $0)/install/can_gateway/lib/can_gateway/can_gateway_node --ros-args -r __node:=can_gateway --params-file <(echo "can_gateway:
  ros__parameters:
    interface: vcan0")
