# Dependencies

> See [README.md](README.md) for quick start instructions.

Run `./setup.sh` to install all dependencies automatically.

## Manual Installation

```bash
# System packages
sudo apt install -y build-essential cmake git python3-colcon-common-extensions can-utils

# ROS 2 dependencies
rosdep install --from-paths src --ignore-src -r -y
```