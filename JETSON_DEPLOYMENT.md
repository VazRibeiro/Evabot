# Jetson Deployment

> See [README.md](README.md) for general setup instructions.

Quick setup for Nvidia Jetson AGX Orin.

## Setup

```bash
# Create service user
sudo useradd -r -s /bin/false -d /opt/evabot evabot
sudo usermod -a -G dialout evabot
sudo mkdir -p /opt/evabot && sudo chown evabot:evabot /opt/evabot

# Deploy workspace
cd /opt/evabot
sudo -u evabot git clone <repo-url> .
sudo -u evabot colcon build

# Install service
sudo cp src/can_gateway/config/can_gateway.service /etc/systemd/system/
sudo systemctl enable --now can_gateway.service
```

## Hardware CAN

```bash
# Configure CAN interface
sudo ip link set can0 up type can bitrate 500000

# Update service to use hardware CAN
sudo systemctl edit can_gateway.service
# Add: Environment="CAN_INTERFACE=can0"
```

## Monitoring

```bash
# Service status
sudo systemctl status can_gateway.service
journalctl -u can_gateway.service -f

# System resources
jtop  # Jetson-specific monitor
```