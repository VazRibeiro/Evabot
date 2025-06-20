#!/usr/bin/env bash
set -e

# Enable logging
exec 1> >(logger -s -t can_gateway_startup)
exec 2>&1

echo "Starting CAN Gateway..."
echo "Working directory: $(pwd)"
echo "User: $(whoami)"

# Create vcan0 if it doesn't exist
if ! ip link show vcan0 >/dev/null 2>&1; then
    echo "Creating vcan0 interface..."
    sudo ip link add dev vcan0 type vcan || true
    sudo ip link set up vcan0 || true
fi

# Source ROS 2 environment
echo "Sourcing ROS 2 environment..."
source /opt/ros/humble/setup.bash

# Source workspace (adjust path for your deployment)
echo "Sourcing workspace..."
source $(pwd)/install/setup.bash

# Set additional ROS 2 environment variables
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "Starting CAN gateway node directly..."

# Start the CAN gateway node and configure/activate it directly
exec $(pwd)/install/can_gateway/lib/can_gateway/can_gateway_node \
    --ros-args -r __node:=can_gateway \
    -p interface:=vcan0
