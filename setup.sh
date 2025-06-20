#!/usr/bin/env bash
# Evabot Workspace Setup Script
# This script installs all required dependencies and sets up the development environment

set -e

echo "🤖 Evabot Workspace Setup"
echo "=========================="

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

# Check if running on Ubuntu
if ! command -v apt &> /dev/null; then
    print_error "This script is designed for Ubuntu/Debian systems with apt package manager"
    exit 1
fi

# Update package list
print_status "Updating package list..."
sudo apt update

# Install system dependencies
print_status "Installing system dependencies..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-vcstool \
    python3-rosdep \
    python3-colcon-common-extensions \
    can-utils \
    iproute2 \
    linux-headers-$(uname -r) \
    clang-format \
    gdb \
    htop

print_success "System dependencies installed"

# Check if ROS 2 Humble is installed
if ! command -v ros2 &> /dev/null; then
    print_warning "ROS 2 not found. Please install ROS 2 Humble manually:"
    echo "  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    echo ""
    print_warning "After installing ROS 2, add this to your ~/.bashrc:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  export ROS_DOMAIN_ID=13"
else
    print_success "ROS 2 found: $(ros2 --version)"
fi

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    print_status "Initializing rosdep..."
    sudo rosdep init
fi

print_status "Updating rosdep..."
rosdep update

# Install ROS dependencies for the workspace
if [ -d "src" ]; then
    print_status "Installing ROS package dependencies..."
    rosdep install --from-paths src --ignore-src -r -y
    print_success "ROS dependencies installed"
else
    print_warning "No 'src' directory found. Run this script from the workspace root after importing packages."
fi

# Load vcan kernel module
print_status "Loading virtual CAN kernel module..."
if sudo modprobe vcan; then
    print_success "vcan module loaded"
else
    print_warning "Failed to load vcan module. You may need to install linux-modules-extra."
fi

# Set up CAN interface permissions
print_status "Setting up CAN interface permissions..."
if ! groups $USER | grep -q dialout; then
    print_status "Adding user to dialout group for CAN access..."
    sudo usermod -a -G dialout $USER
    print_warning "You need to log out and log back in for group changes to take effect"
else
    print_success "User already in dialout group"
fi

# Make scripts executable
print_status "Making scripts executable..."
chmod +x configure_gateway.sh start_can_gateway.sh 2>/dev/null || true

# Check workspace setup
print_status "Checking workspace setup..."
if [ -f "Evabot.repos" ]; then
    print_success "Evabot.repos found"
    
    if [ ! -d "src" ] || [ -z "$(ls -A src 2>/dev/null)" ]; then
        print_warning "Source directory is empty. Import packages with:"
        echo "  vcs import src < Evabot.repos"
    fi
else
    print_warning "Evabot.repos not found. Make sure you're in the workspace root."
fi

if [ -d "src" ] && [ "$(ls -A src 2>/dev/null)" ]; then
    if [ ! -d "install" ]; then
        print_warning "Workspace not built. Build with:"
        echo "  colcon build"
    else
        print_success "Workspace appears to be built"
    fi
fi

echo ""
print_success "Setup complete! 🎉"
echo ""
echo "Next steps:"
echo "1. If you haven't already, import packages: vcs import src < Evabot.repos"
echo "2. Build the workspace: colcon build"
echo "3. Source the workspace: source install/setup.bash"
echo "4. Test the CAN gateway: ros2 launch can_gateway gateway.launch.py"
echo ""
echo "For systemd service setup, see the README.md"
