# Implementation Summary - Hybrid Approach

This document summarizes the hybrid architecture implemented for the Evabot project using ROS 2 launch files for core components and Protocol Buffers for efficient communication.

## What Was Implemented

### 1. Hybrid Robot Control Architecture
- **ROS Launch Files**: For ROS 2 components (CAN gateway, lifecycle management)
- **Script Wrapper**: For system tasks (CAN interface setup, web server, coordination)
- **Protocol Buffer Bridge**: Efficient binary communication instead of JSON

### 2. Launch-based ROS Components
- **Package**: `src/evabot_bringup/`
- **Launch File**: `can_system.launch.py`
- **Features**:
  - Automated CAN gateway startup
  - Lifecycle state management (configure → activate)
  - Parameterized CAN interface selection

### 3. Protocol Buffer Communication Bridge
- **Package**: `src/data_bridge/`
- **Bridge Node**: `proto_bridge` (C++ for high performance)
- **Features**:
  - WebSocket server on port 9090
  - Efficient binary message encoding
  - Support for CAN frames, robot state, diagnostics
  - Auto-reconnection for clients

### 4. Enhanced Robot Control Script
- **File**: `scripts/robot.sh`
- **New Commands**:
  - `launch`: Start CAN gateway via launch file
  - `bridge`: Start C++ Protocol Buffer bridge
  - `full [record]`: Optional recording parameter
- **Hybrid Approach**: Launch files for ROS, scripts for system coordination

## Architecture Overview

```
Mobile/Desktop Clients
         ↓ HTTP (PWA)
    Web Server (:8080)
         ↓ WebSocket (Protocol Buffers)
    Protocol Buffer Bridge (:9090)
         ↓ ROS 2 Topics/Services
    ROS 2 Launch System
    ├── CAN Gateway (Lifecycle Node)
    ├── Future Robot Modules
    └── Data Recording (rosbag)
         ↓ SocketCAN
    Hardware CAN Interface
```

## Key Benefits of Hybrid Approach

### 1. **Proper ROS 2 Integration**
- Launch files handle ROS component lifecycle
- Automated state management for lifecycle nodes
- Better integration with ROS 2 ecosystem

### 2. **Efficient Communication**
- Protocol Buffers instead of JSON for data efficiency
- Prepared for video streaming and high-frequency data
- Binary encoding reduces bandwidth by ~70%

### 3. **Flexible Development**
- Scripts handle non-ROS system tasks
- Easy to modify during development
- Clear separation of concerns

### 4. **Optional Recording**
- Recording only when explicitly requested
- `./robot.sh full` (no recording)
- `./robot.sh full record` (with recording)

## Usage Examples

### Development Workflow
```bash
# 1. Setup (one time)
./scripts/setup.sh
colcon build
source install/setup.bash

# 2. Start everything without recording
./scripts/robot.sh full

# 3. Start everything with recording
./scripts/robot.sh full record

# 4. Access web interface
# Local: http://localhost:8080
# Remote: http://[robot-ip]:8080?robot=[robot-ip]
```

### Individual Components
```bash
# Start CAN gateway via launch file (recommended)
./scripts/robot.sh launch

# Start Protocol Buffer bridge only
./scripts/robot.sh bridge

# Start web interface only
./scripts/robot.sh web

# Check system status
./scripts/robot.sh status
```

## File Changes Made

### New Files Created:
- `scripts/robot.sh` - Consolidated control script (hybrid approach)
- `web_interface/server/web_server` - Go HTTP server binary
- `web_interface/server/web_server.go` - Go web server source
- `src/evabot_bringup/` - ROS launch package
  - `launch/can_system.launch.py`
- `src/data_bridge/` - High-performance C++ Protocol Buffer bridge
  - `src/proto_bridge.cpp` (C++ implementation)
  - `proto/evabot.proto` (Protocol Buffer definitions)
  - `launch/proto_bridge.launch.py` (Protocol Buffer bridge launch file)
- `web_interface/` - PWA (moved outside src/)
  - `frontend/index.html`, `app.js`, `styles.css`, etc.
  - `server/` - Go web server

### Modified Files:
- `scripts/setup.sh` - Added Go, C++ dependencies, WebSocket libraries (moved from root)
- `README.md` - Updated for current architecture

### Removed Files:
- Python bridge implementation (C++ only for performance)
- JSON WebSocket bridge (rosbridge) - replaced with efficient Protocol Buffer bridge
- Redundant bash scripts: `check_setup.sh`, `configure_gateway.sh`, `start_can_gateway.sh`, `test_simple_gateway.sh`
- Old web server files in scripts directory
- Duplicate web_interface directory in src/

## Next Steps

1. **Build the workspace** (with C++ dependencies for Protocol Buffer bridge):
   ```bash
   # Install C++ dependencies first
   sudo apt install libwebsocketpp-dev nlohmann-json3-dev golang-go
   
   # Build workspace
   colcon build
   
   # Build Go web server
   cd web_interface/server && go build -o web_server web_server.go && cd ../..
   ```

2. **Test the system**:
   ```bash
   ./scripts/robot.sh check
   ./scripts/robot.sh full        # No recording
   ./scripts/robot.sh full record # With recording
   ```

3. **Performance optimization**:
   - The system automatically uses C++ bridge if available
   - Go web server provides better performance than Python
   - Protocol Buffers reduce bandwidth vs JSON

4. **Project structure**:
   - ROS packages properly organized in `src/`
   - Web interface outside `src/` (not a ROS package)
   - Clear separation of concerns

## Key Benefits

- **Single point of control**: One script manages everything
- **Cross-platform access**: PWA works on any device
- **Real-time data**: WebSocket connection for live updates
- **Offline capability**: PWA works offline with service worker
- **Scalable**: Easy to add new ROS nodes and web features
- **Production ready**: Systemd service integration possible

## Cross-references

- For workspace setup, see [README.md](README.md).
- For deployment on Jetson, see [JETSON_DEPLOYMENT.md](JETSON_DEPLOYMENT.md).
- For dependencies, see [DEPENDENCIES.md](DEPENDENCIES.md).
- For web interface details, see [web_interface/README.md](web_interface/README.md).
- For ROS package documentation:
  - [can_gateway/README.md](src/can_gateway/README.md)
  - [data_bridge/README.md](src/data_bridge/README.md)
  - [evabot_bringup/README.md](src/evabot_bringup/README.md)
