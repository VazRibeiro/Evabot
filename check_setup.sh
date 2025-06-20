#!/usr/bin/env bash
# Simple deployment check

set -e

echo "Checking Evabot deployment..."

# Check ROS environment
if [[ -z "$ROS_DISTRO" ]]; then
    echo "❌ ROS not sourced. Run: source /opt/ros/humble/setup.bash"
    exit 1
fi
echo "✅ ROS $ROS_DISTRO"

# Check workspace
if [[ -f "install/can_gateway/lib/can_gateway/can_gateway_node" ]]; then
    echo "✅ Workspace built"
else
    echo "❌ Workspace not built. Run: colcon build"
    exit 1
fi

# Check CAN interface
if ip link show | grep -q "can"; then
    echo "✅ CAN interface available"
else
    echo "⚠️  No CAN interface. For testing: sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0"
fi

# Test basic functionality
if ros2 pkg list | grep -q can_gateway; then
    echo "✅ Package accessible"
else
    echo "❌ Package not found. Run: source install/setup.bash"
    exit 1
fi

echo "✅ Basic deployment checks passed"
