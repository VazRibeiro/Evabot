# Evabot

ROS 2 workspace for the Evabot autonomous agricultural platform.

## Quick Start

```bash
# Setup workspace
mkdir -p ~/evabot_ws && cd ~/evabot_ws
git clone <repo-url> .
./setup.sh

# Build and run
colcon build
source install/setup.bash
ros2 launch can_gateway gateway.launch.py
```

## Packages

- **can_gateway**: CAN ↔ ROS 2 bridge with SocketCAN support (see [src/can_gateway/README.md](src/can_gateway/README.md))

## Requirements

- Ubuntu 22.04+ with ROS 2 Humble
- CAN interface (hardware or virtual)
- Standard build tools (see [DEPENDENCIES.md](DEPENDENCIES.md) for manual setup)

## Deployment

### Development
```bash
# Virtual CAN for testing
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
ros2 launch can_gateway gateway.launch.py
```

### Production (Jetson/Robot)
```bash
# Hardware CAN setup
sudo ip link set can0 up type can bitrate 500000
sudo cp src/can_gateway/config/can_gateway.service /etc/systemd/system/
sudo systemctl enable --now can_gateway.service
```

> For detailed Jetson AGX Orin setup, see [JETSON_DEPLOYMENT.md](JETSON_DEPLOYMENT.md)

## Environment

Add to `~/.bashrc`:
```bash
export ROS_DOMAIN_ID=13
source /opt/ros/humble/setup.bash
source ~/evabot_ws/install/setup.bash  # adjust path for deployment
```

## Troubleshooting

```bash
# Check setup
./check_setup.sh

# Common issues
sudo usermod -a -G dialout $USER  # CAN permissions
source install/setup.bash         # Package not found
sudo systemctl status can_gateway.service  # Service issues
```
