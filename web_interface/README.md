# Evabot Web Interface

Progressive Web App (PWA) for remote monitoring and control of the Evabot system.

## Structure

- `frontend/`: PWA application files
  - `index.html`: Main application HTML
  - `app.js`: JavaScript application logic
  - `styles.css`: CSS styling
  - `manifest.json`: PWA manifest
  - `service-worker.js`: Service worker for offline functionality
  - `icons/`: Application icons
- `server/`: Go HTTP server
  - `web_server.go`: Go server source
  - `web_server`: Compiled binary

## Usage

### Automatic (Recommended)
```bash
# Start complete system including web server
./scripts/robot.sh full

# Or just web server
./scripts/robot.sh web
```

### Manual
```bash
# Build web server (if not already done)
cd web_interface/server
go build -o web_server web_server.go

# Start web server
./web_server 8080

# Access at http://localhost:8080
```

### Remote Access
For remote access, access via robot's IP: `http://[robot-ip]:8080`

## PWA Features

- Installable on mobile devices and desktop
- Offline capability with service worker
- Responsive design for all screen sizes
- Real-time ROS 2 topic visualization
- Robot control interfaces

## Icons

You'll need to add:
- `icons/robot-icon-192.png` (192x192 px)
- `icons/robot-icon-512.png` (512x512 px)

These can be simple robot icons or the Evabot logo.
