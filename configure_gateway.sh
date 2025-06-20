#!/usr/bin/env bash
set -e

echo "Configuring and testing CAN Gateway..."

# Configure the CAN gateway
echo "1. Configuring the CAN gateway..."
timeout 5 ros2 service call /can_gateway/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"

# Activate the CAN gateway  
echo "2. Activating the CAN gateway..."
timeout 5 ros2 service call /can_gateway/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"

# Check the state
echo "3. Checking final state..."
timeout 5 ros2 service call /can_gateway/get_state lifecycle_msgs/srv/GetState

echo "4. Checking available topics..."
ros2 topic list | grep can

echo "5. Sending test CAN message and listening for 5 seconds..."
timeout 5 ros2 topic echo /can/raw &
sleep 1
cansend vcan0 789#DEADCAFE
echo "Test CAN message sent: 789#DEADCAFE"
wait

echo "CAN Gateway configuration complete!"