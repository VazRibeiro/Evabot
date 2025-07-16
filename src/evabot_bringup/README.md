# Evabot Bringup Package

System-wide launch files for coordinated startup of Evabot components.

## Overview

The evabot_bringup package provides ROS 2 launch files for properly coordinated system startup, lifecycle management, and multi-node orchestration.

## Features

- **Lifecycle Management**: Proper node initialization and shutdown
- **Parameter Configuration**: Centralized parameter management
- **Multi-node Coordination**: Synchronized startup of dependent services
- **Environment Setup**: Automatic CAN interface configuration

## Launch Files

### can_system.launch.py

Primary system launch file for CAN gateway and related components.

```bash
# Basic launch
ros2 launch evabot_bringup can_system.launch.py

# Custom CAN interface
ros2 launch evabot_bringup can_system.launch.py interface:=can0

# Debug mode
ros2 launch evabot_bringup can_system.launch.py debug:=true
```

**Parameters:**
- `interface` (default: vcan0) - CAN interface name
- `debug` (default: false) - Enable debug output

**Launched Nodes:**
- CAN Gateway (can_gateway package)
- Lifecycle management services
- Diagnostic monitoring

## Usage

### Development
```bash
# Start complete CAN system
ros2 launch evabot_bringup can_system.launch.py

# Or use the robot control script
./scripts/robot.sh launch
```

### Production
```bash
# Start with real hardware interface
ros2 launch evabot_bringup can_system.launch.py interface:=can0
```

## Integration

This package integrates with:

- **[CAN Gateway](../can_gateway/README.md)** - CAN bus communication
- **[Data Bridge](../data_bridge/README.md)** - WebSocket data streaming
- **Robot Control Script** - `./scripts/robot.sh launch`

## Configuration

Launch files automatically:
1. Configure CAN interface parameters
2. Set up ROS 2 environment variables
3. Initialize lifecycle nodes in correct order
4. Enable system monitoring

## Development

### Adding New Launch Files

1. Create launch file in `launch/` directory
2. Follow ROS 2 launch file conventions
3. Update this documentation

### Parameter Configuration

Common parameters are defined in launch files. For custom configurations:

```bash
# Override parameters
ros2 launch evabot_bringup can_system.launch.py \
  interface:=can1 \
  debug:=true
```

## Troubleshooting

### Launch Issues
```bash
# Check launch file syntax
ros2 launch --show-args evabot_bringup can_system.launch.py

# Debug launch process
ros2 launch evabot_bringup can_system.launch.py debug:=true
```

### Node Dependencies
```bash
# Check node status
ros2 node list
ros2 lifecycle list

# Check parameter values
ros2 param list /can_gateway
```

## See Also

- [CAN Gateway Package](../can_gateway/README.md) - Core CAN functionality
- [Robot Control Script](../../scripts/robot.sh) - System management
- [ROS 2 Launch Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)

## Cross-references

- For workspace setup, see [../../README.md](../../README.md).
- For CAN gateway details, see [can_gateway/README.md](../can_gateway/README.md).
- For data bridge details, see [data_bridge/README.md](../data_bridge/README.md).
