#!/usr/bin/env bash
# Evabot Robot Control Script
# Consolidated script for managing the robot system

set -e

# Enable logging
exec 1> >(logger -s -t evabot_robot)
exec 2>&1

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Get script directory (works from anywhere)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

function check_environment() {
    print_status "Checking Evabot environment..."
    
    # Check ROS environment
    if [[ -z "$ROS_DISTRO" ]]; then
        print_status "Sourcing ROS environment..."
        source /opt/ros/humble/setup.bash
    fi
    print_success "ROS $ROS_DISTRO"
    
    # Check workspace
    if [[ ! -f "$WORKSPACE_DIR/install/can_gateway/lib/can_gateway/can_gateway_node" ]]; then
        print_error "Workspace not built. Run: colcon build"
        exit 1
    fi
    print_success "Workspace built"
    
    # Source workspace if needed
    if ! ros2 pkg list | grep -q can_gateway; then
        print_status "Sourcing workspace..."
        source "$WORKSPACE_DIR/install/setup.bash"
    fi
    print_success "Package accessible"
    
    # Set environment variables
    export ROS_DOMAIN_ID=13
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    print_success "Environment configured"
}

function setup_can_interface() {
    print_status "Setting up CAN interface..."
    
    # Create vcan0 if it doesn't exist
    if ! ip link show vcan0 >/dev/null 2>&1; then
        print_status "Creating vcan0 interface..."
        sudo ip link add dev vcan0 type vcan || true
        sudo ip link set up vcan0 || true
    fi
    print_success "CAN interface available"
}

function start_gateway() {
    print_status "Starting CAN gateway node..."
    cd "$WORKSPACE_DIR"
    exec "$WORKSPACE_DIR/install/can_gateway/lib/can_gateway/can_gateway_node" \
        --ros-args -r __node:=can_gateway \
        -p interface:=vcan0
}

function start_gateway_launch() {
    print_status "Starting CAN gateway via launch file..."
    cd "$WORKSPACE_DIR"
    ros2 launch evabot_bringup can_system.launch.py interface:=vcan0
}

function start_bridge() {
    print_status "Starting C++ Protocol Buffer bridge..."
    cd "$WORKSPACE_DIR"
    
    # Use C++ bridge for optimal performance
    if [[ -f "$WORKSPACE_DIR/install/data_bridge/lib/data_bridge/proto_bridge" ]]; then
        ros2 run data_bridge proto_bridge
    else
        print_error "C++ Protocol Buffer bridge not found. Please build the workspace:"
        print_error "  colcon build"
        exit 1
    fi
}

function start_recording() {
    print_status "Starting data recording..."
    cd "$WORKSPACE_DIR"
    
    # Create logs directory if it doesn't exist
    mkdir -p logs/rosbags
    
    # Start recording with timestamp
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    ros2 bag record -o "logs/rosbags/evabot_$TIMESTAMP" \
        /can/raw /can/tx /can/rx \
        /robot_state /diagnostics \
        --compression-mode file --compression-format zstd
}

function start_web_server() {
    print_status "Starting Go web server..."
    cd "$WORKSPACE_DIR"
    
    # Try different locations for the web server
    if [[ -f "$WORKSPACE_DIR/web_interface/server/web_server" ]]; then
        "$WORKSPACE_DIR/web_interface/server/web_server" 8080
    elif [[ -f "$WORKSPACE_DIR/web_interface/web_server" ]]; then
        cd "$WORKSPACE_DIR/web_interface"
        ./web_server 8080
    else
        print_error "Web server binary not found. Please build it:"
        print_error "  cd web_interface/server && go build -o web_server web_server.go"
        exit 1
    fi
}

function configure_gateway() {
    print_status "Configuring and testing CAN Gateway..."
    
    # Configure the CAN gateway
    print_status "1. Configuring the CAN gateway..."
    timeout 10 ros2 service call /can_gateway/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}" || {
        print_warning "Gateway configuration may have timed out, continuing..."
    }
    
    # Activate the CAN gateway  
    print_status "2. Activating the CAN gateway..."
    timeout 10 ros2 service call /can_gateway/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}" || {
        print_warning "Gateway activation may have timed out, continuing..."
    }
    
    # Check the state
    print_status "3. Checking final state..."
    timeout 10 ros2 service call /can_gateway/get_state lifecycle_msgs/srv/GetState || {
        print_warning "State check may have timed out"
    }
    
    print_success "CAN Gateway configuration complete!"
}

function test_gateway() {
    print_status "Testing CAN Gateway..."
    
    print_status "1. Checking available topics..."
    ros2 topic list | grep can || print_warning "No CAN topics found"
    
    print_status "2. Sending test CAN message and listening for 5 seconds..."
    timeout 5 ros2 topic echo /can/raw &
    sleep 1
    if command -v cansend >/dev/null 2>&1; then
        cansend vcan0 789#DEADCAFE
        print_success "Test CAN message sent: 789#DEADCAFE"
    else
        print_warning "cansend not available, skipping test message"
    fi
    wait
    
    print_success "CAN Gateway test complete!"
}

function show_status() {
    print_status "Evabot System Status:"
    echo "====================="
    
    # Check ROS nodes
    if ros2 node list | grep -q can_gateway; then
        print_success "CAN Gateway: Running"
    else
        print_warning "CAN Gateway: Not running"
    fi
    
    # Check bridge
    if ros2 node list | grep -q proto_bridge; then
        print_success "Protocol Buffer Bridge: Running"
    else
        print_warning "Protocol Buffer Bridge: Not running"
    fi
    
    # Check CAN interface
    if ip link show | grep -q "vcan0.*UP"; then
        print_success "CAN Interface: UP"
    else
        print_warning "CAN Interface: DOWN"
    fi
    
    # Check topics
    echo ""
    print_status "Available CAN topics:"
    ros2 topic list | grep can || echo "  None"
}

function show_help() {
    echo "Evabot Robot Control Script - Hybrid Launch Approach"
    echo "Usage: $0 {command}"
    echo ""
    echo "Commands:"
    echo "  check         - Check environment and dependencies"
    echo "  gateway       - Start CAN gateway only (direct node)"
    echo "  launch        - Start CAN gateway via launch file"
    echo "  bridge        - Start C++ Protocol Buffer bridge"
    echo "  record        - Start data recording (rosbag)"
    echo "  web           - Start web server for PWA"
    echo "  configure     - Configure and activate gateway"
    echo "  test          - Test gateway functionality"
    echo "  status        - Show system status"
    echo "  full [record] - Start complete system (launch + bridge + web + optional recording)"
    echo "  help          - Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 check              # Validate setup"
    echo "  $0 launch             # Start CAN gateway via launch file"
    echo "  $0 full               # Start everything without recording"
    echo "  $0 full record        # Start everything with recording"
}

# Function to check and cleanup ports
cleanup_ports() {
    local ports=("8080" "9090")
    
    for port in "${ports[@]}"; do
        local pid=$(lsof -ti:$port 2>/dev/null || true)
        if [[ -n "$pid" ]]; then
            print_warning "Port $port is in use by process $pid. Stopping it..."
            kill $pid 2>/dev/null || true
            sleep 1
            # Force kill if still running
            if kill -0 $pid 2>/dev/null; then
                print_warning "Force killing process $pid on port $port"
                kill -9 $pid 2>/dev/null || true
            fi
        fi
    done
}

# Main command router
case "${1:-help}" in
    check)
        check_environment
        setup_can_interface
        print_success "Environment check complete!"
        ;;
    gateway)
        check_environment
        setup_can_interface
        start_gateway
        ;;
    bridge)
        check_environment
        start_bridge
        ;;
    launch)
        check_environment
        setup_can_interface
        start_gateway_launch
        ;;
    record)
        check_environment
        start_recording
        ;;
    web)
        start_web_server
        ;;
    configure)
        check_environment
        configure_gateway
        ;;
    test)
        check_environment
        test_gateway
        ;;
    status)
        check_environment
        show_status
        ;;
    full)
        check_environment
        setup_can_interface
        cleanup_ports
        
        # Start ROS nodes via launch file in background
        print_status "Starting CAN gateway via launch file in background..."
        ros2 launch evabot_bringup can_system.launch.py interface:=vcan0 &
        LAUNCH_PID=$!
        
        # Wait for nodes to initialize
        sleep 5
        
        # Start Protocol Buffer bridge in background
        print_status "Starting C++ Protocol Buffer bridge in background..."
        if [[ -f "$WORKSPACE_DIR/install/data_bridge/lib/data_bridge/proto_bridge" ]]; then
            ros2 run data_bridge proto_bridge &
        else
            print_error "C++ Protocol Buffer bridge not found. Please build the workspace."
            exit 1
        fi
        BRIDGE_PID=$!
        
        # Conditionally start recording
        if [[ "$2" == "record" ]]; then
            print_status "Starting data recording in background..."
            start_recording &
            RECORD_PID=$!
            print_status "Recording PID: $RECORD_PID"
        else
            print_status "Recording disabled (use './robot.sh full record' to enable)"
            RECORD_PID=""
        fi
        
        # Start web server in background
        print_status "Starting Go web server in background..."
        if [[ -f "$WORKSPACE_DIR/web_interface/server/web_server" ]]; then
            "$WORKSPACE_DIR/web_interface/server/web_server" 8080 &
        elif [[ -f "$WORKSPACE_DIR/web_interface/web_server" ]]; then
            cd "$WORKSPACE_DIR/web_interface"
            ./web_server 8080 &
            cd "$WORKSPACE_DIR"
        else
            print_error "Web server binary not found. Skipping web interface."
            WEB_PID=""
        fi
        if [[ -n "$!" ]]; then
            WEB_PID=$!
        else
            WEB_PID=""
        fi
        
        print_success "Full system started!"
        print_status "Launch PID: $LAUNCH_PID"
        print_status "Bridge PID: $BRIDGE_PID" 
        if [[ -n "$RECORD_PID" ]]; then
            print_status "Recording PID: $RECORD_PID"
        fi
        print_status "Web Server PID: $WEB_PID"
        print_status "🌐 Web interface: http://localhost:8080"
        print_status "🔌 Protocol Buffer WebSocket: ws://localhost:9090"
        print_status "Press Ctrl+C to stop all processes"
        
        # Wait for user interrupt
        if [[ -n "$RECORD_PID" ]]; then
            trap 'kill $LAUNCH_PID $BRIDGE_PID $RECORD_PID $WEB_PID 2>/dev/null; exit' INT
        else
            trap 'kill $LAUNCH_PID $BRIDGE_PID $WEB_PID 2>/dev/null; exit' INT
        fi
        wait
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        print_error "Unknown command: $1"
        show_help
        exit 1
        ;;
esac
