# Evabot

ROS 2 workspace for the Evabot autonomous agricultural platform.

---

## Table of Contents
- [Overview](#overview)
- [Requirements](#requirements)
- [Importing ROS Packages](#importing-ros-packages)
- [Quick Start](#quick-start)
- [Deployment](#deployment)
- [Environment Setup](#environment-setup)
- [Troubleshooting](#troubleshooting)
- [ROS Packages](#ros-packages)
- [Web Interface](#web-interface)

---

## Overview
Evabot is a modular, ROS 2-based robot system for autonomous agricultural tasks. It features:
- CAN ↔ ROS 2 bridge (SocketCAN)
- High-performance C++ Protocol Buffer WebSocket bridge
- Progressive Web App (PWA) for remote monitoring
- System-wide launch and management scripts

---

## Requirements
- Ubuntu 22.04+ with ROS 2 Humble
- CAN interface (hardware or virtual)
- Go (for web server)
- C++ development tools (Protocol Buffer bridge)
- Modern web browser (for PWA)
- Standard build tools

### System Packages
Install manually if not using `setup.sh`:
```bash
sudo apt install -y build-essential cmake git python3-colcon-common-extensions can-utils
```

### ROS 2 Dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## Importing ROS Packages
Before building, import all required ROS packages:
```bash
vcs import src < Evabot.repos
rosdep install --from-paths src --ignore-src -r -y
```

---

## Quick Start
```bash
# Setup workspace
mkdir -p ~/evabot_ws && cd ~/evabot_ws
git clone <repo-url> .
./scripts/setup.sh

# Build
colcon build
source install/setup.bash

# Start complete system
./scripts/robot.sh full

# Or start components individually
./scripts/robot.sh gateway    # CAN gateway only
./scripts/robot.sh bridge     # Data bridge only
```

---

## Deployment
### Development
```bash
# Virtual CAN for testing
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
./scripts/robot.sh full
# Access web interface
# Local: http://localhost:8080
# Remote: http://[robot-ip]:8080?robot=[robot-ip]
```

### Production (Jetson/Robot)
#### Create Service User
```bash
sudo useradd -r -s /bin/false -d /opt/evabot evabot
sudo usermod -a -G dialout evabot
sudo mkdir -p /opt/evabot && sudo chown evabot:evabot /opt/evabot
```

#### Deploy Workspace
```bash
cd /opt/evabot
sudo -u evabot git clone <repo-url> .
sudo -u evabot colcon build
```

#### Install and Configure Service
```bash
sudo cp src/can_gateway/config/can_gateway.service /etc/systemd/system/
sudo systemctl enable --now can_gateway.service
```

#### Hardware CAN Setup
```bash
sudo ip link set can0 up type can bitrate 500000
sudo systemctl edit can_gateway.service
# Add: Environment="CAN_INTERFACE=can0"
```

#### Monitoring
```bash
sudo systemctl status can_gateway.service
journalctl -u can_gateway.service -f
jtop  # Jetson-specific monitor
```

---

## Environment Setup
Add to `~/.bashrc`:
```bash
export ROS_DOMAIN_ID=13
source /opt/ros/humble/setup.bash
source ~/evabot_ws/install/setup.bash  # adjust path for deployment
```

---

## Troubleshooting
```bash
./scripts/robot.sh check  # Check setup
sudo usermod -a -G dialout $USER  # CAN permissions
source install/setup.bash         # Package not found
sudo systemctl status can_gateway.service  # Service issues
```

---

## ROS Packages
### can_gateway
- **Description**: CAN ↔ ROS 2 bridge with SocketCAN support.
- **Topics**:
  - `/can/raw`: CAN frames from bus.
  - `/can/tx`: CAN frames to bus.
- **Parameters**:
  - `interface`: CAN interface name (default: `vcan0`).
  - `rx_rate_hz`: Polling rate (default: `10000`).

### data_bridge
- **Description**: High-performance C++ Protocol Buffer WebSocket bridge.
- **Topics**:
  - `/can/raw`: Raw CAN frames.
  - `/diagnostics`: System diagnostic information.
  - `/robot_state`: Robot status messages.

### evabot_bringup
- **Description**: System-wide launch files for coordinated startup.
- **Launch Files**:
  - `can_system.launch.py`: Primary system launch file.
  - Parameters:
    - `interface`: CAN interface name (default: `vcan0`).
    - `debug`: Enable debug output (default: `false`).

---

## Web Interface
- **web_interface/**: Progressive Web App for remote monitoring
- **web_interface/server/**: Go-based HTTP server for serving the PWA

See [web_interface/README.md](web_interface/README.md) for details.
