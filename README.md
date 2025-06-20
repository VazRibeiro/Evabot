# Evabot

This repository contains the workspace for the navigation software controlling the Evabot platform.

This workspace uses `vcs` (version control system) tool to manage multiple ROS2 packages as separate repositories.

## Packages included:
- can_gateway: reads and publishes CAN messages in topics. Sends CAN messages from topics.

## Setup

### Prerequisites
- ROS2 (tested with humble/iron)
- vcs tool: `sudo apt install python3-vcstool` or `pip install vcstool`

### Install

From the root of the ROS workspace:

```bash
# ---- Import all packages ----
vcs import src < Evabot.repos

# ---- Install dependencies ----
rosdep install --from-paths src --ignore-src -r -y

# ---- Build ----
colcon build
source install/setup.bash

# ---- Simulate CAN on desktop ----
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# ---- Run gateway ----
ros2 launch can_gateway gateway.launch.py

# ---- Send a test frame ----
cansend vcan0 123#DEADBEEF

# ---- Echo ROS topic ----
ros2 topic echo /can/raw
```

## Adding new packages

To add a new package to this workspace:

1. Create your package repository on GitHub/GitLab
2. Add it to `Evabot.repos`:
   ```yaml
   repositories:
     your_new_package:
       type: git
       url: https://github.com/your-username/your_new_package.git
       version: main
   ```
3. Run `vcs import src < Evabot.repos` to fetch the new package
4. Build with `colcon build`
