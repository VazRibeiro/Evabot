# Evabot

ROS 2 workspace for the Evabot autonomous agricultural platform.

## Quick Start

```bash
# Setup workspace
mkdir -p ~/evabot_ws && cd ~/evabot_ws
git clone <repo-url> .
./scripts/setup.sh

# Build and run
colcon build
source install/setup.bash

# Start complete system (gateway + bridge + recording)
./scripts/robot.sh full

# Or start components individually:
./scripts/robot.sh gateway    # CAN gateway only
./scripts/robot.sh bridge     # Data bridge only
```

## Packages

- **can_gateway**: CAN ↔ ROS 2 bridge with SocketCAN support (see [src/can_gateway/README.md](src/can_gateway/README.md))
- **data_bridge**: High-performance C++ Protocol Buffer WebSocket bridge
- **evabot_bringup**: ROS 2 launch files for system startup

## Web Interface

- **web_interface/**: Progressive Web App for remote monitoring (outside src - not a ROS package)
- **web_interface/server/**: Go-based HTTP server for serving the PWA

## Requirements

- Ubuntu 22.04+ with ROS 2 Humble
- CAN interface (hardware or virtual) 
- Go (for web server)
- C++ development tools for Protocol Buffer bridge
- Modern web browser for PWA interface
- Standard build tools (see [DEPENDENCIES.md](DEPENDENCIES.md) for manual setup)

## Deployment

### Development
```bash
# Virtual CAN for testing
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0

# Start system components
./scripts/robot.sh full

# Access web interface
# Local: http://localhost:8080 (after starting web server)
# Remote via NetBird: http://[robot-ip]:8080?robot=[robot-ip]
```

### Production (Jetson/Robot)
```bash
# Hardware CAN setup
sudo ip link set can0 up type can bitrate 1000000
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
./scripts/robot.sh check

# Common issues
sudo usermod -a -G dialout $USER  # CAN permissions
source install/setup.bash         # Package not found
sudo systemctl status can_gateway.service  # Service issues
```
