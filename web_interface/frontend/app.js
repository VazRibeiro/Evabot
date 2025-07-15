// Evabot PWA Application - Protocol Buffer Version
class EvabotApp {
    constructor() {
        this.ws = null;
        this.connected = false;
        this.canMessageCount = 0;
        this.canRateCounter = 0;
        this.lastRateUpdate = Date.now();
        
        this.initializeUI();
        this.setupEventListeners();
        this.updateConnectionStatus('disconnected');
        
        // Try to connect automatically on load
        setTimeout(() => this.connect(), 1000);
    }

    initializeUI() {
        this.elements = {
            connectionStatus: document.getElementById('connection-status'),
            connectBtn: document.getElementById('connect-btn'),
            configureBtn: document.getElementById('configure-btn'),
            testBtn: document.getElementById('test-btn'),
            canMessages: document.getElementById('can-messages'),
            canCount: document.getElementById('can-count'),
            canRate: document.getElementById('can-rate'),
            robotState: document.getElementById('robot-state'),
            diagnostics: document.getElementById('diagnostics'),
            activityLog: document.getElementById('activity-log'),
            clearLogBtn: document.getElementById('clear-log-btn')
        };
    }

    setupEventListeners() {
        this.elements.connectBtn.addEventListener('click', () => {
            if (this.connected) {
                this.disconnect();
            } else {
                this.connect();
            }
        });

        this.elements.configureBtn.addEventListener('click', () => {
            this.configureGateway();
        });

        this.elements.testBtn.addEventListener('click', () => {
            this.testGateway();
        });

        this.elements.clearLogBtn.addEventListener('click', () => {
            this.clearLog();
        });

        // Update rate counter every second
        setInterval(() => {
            const now = Date.now();
            const elapsed = (now - this.lastRateUpdate) / 1000;
            const rate = Math.round(this.canRateCounter / elapsed);
            this.elements.canRate.textContent = rate;
            this.canRateCounter = 0;
            this.lastRateUpdate = now;
        }, 1000);
    }

    connect() {
        this.updateConnectionStatus('connecting');
        this.log('Attempting to connect to robot...', 'info');

        // Get the robot IP from URL params or use default
        const urlParams = new URLSearchParams(window.location.search);
        const robotIP = urlParams.get('robot') || window.location.hostname || 'localhost';
        const wsUrl = `ws://${robotIP}:9090`;

        this.ws = new WebSocket(wsUrl);

        this.ws.onopen = () => {
            this.connected = true;
            this.updateConnectionStatus('connected');
            this.log('Connected to robot Protocol Buffer bridge!', 'success');
        };

        this.ws.onmessage = (event) => {
            try {
                const message = JSON.parse(event.data);
                this.handleMessage(message);
            } catch (e) {
                this.log(`Error parsing message: ${e.message}`, 'error');
            }
        };

        this.ws.onerror = (error) => {
            this.connected = false;
            this.updateConnectionStatus('disconnected');
            this.log(`Connection error: ${error}`, 'error');
        };

        this.ws.onclose = () => {
            this.connected = false;
            this.updateConnectionStatus('disconnected');
            this.log('Connection closed', 'warning');
            
            // Auto-reconnect after 3 seconds
            setTimeout(() => {
                if (!this.connected) {
                    this.connect();
                }
            }, 3000);
        };
    }

    disconnect() {
        if (this.ws) {
            this.ws.close();
        }
        this.connected = false;
        this.updateConnectionStatus('disconnected');
        this.log('Disconnected from robot', 'info');
    }

    handleMessage(message) {
        switch (message.type) {
            case 'welcome':
                this.log(message.message, 'success');
                break;
                
            case 'can_frame':
                this.handleCanMessage(message.data);
                break;
                
            case 'robot_state':
                this.handleRobotState(message.data);
                break;
                
            case 'diagnostic':
                this.handleDiagnostics(message.data);
                break;
                
            default:
                this.log(`Unknown message type: ${message.type}`, 'warning');
        }
    }

    handleCanMessage(data) {
        this.canMessageCount++;
        this.canRateCounter++;
        this.elements.canCount.textContent = this.canMessageCount;

        // Convert data array to hex string
        const dataHex = data.data
            .map(b => b.toString(16).padStart(2, '0').toUpperCase())
            .join('');

        // Create message element
        const messageElement = document.createElement('div');
        messageElement.className = 'can-message';
        messageElement.innerHTML = `
            <div class="timestamp">${new Date().toLocaleTimeString()}</div>
            <div>ID: <span class="id">0x${data.id.toString(16).toUpperCase()}</span></div>
            <div>Data: <span class="data">${dataHex}</span></div>
            <div>DLC: ${data.dlc} ${data.is_extended ? '(Extended)' : ''}</div>
        `;

        // Add to display (keep only last 20 messages)
        const container = this.elements.canMessages;
        if (container.children.length >= 20) {
            container.removeChild(container.firstChild);
        }
        
        // Remove "no data" message if present
        const noDataMsg = container.querySelector('.no-data');
        if (noDataMsg) {
            noDataMsg.remove();
        }

        container.appendChild(messageElement);
        container.scrollTop = container.scrollHeight;
    }

    handleRobotState(data) {
        let stateDisplay = '';
        
        if (typeof data === 'object') {
            // Format structured robot state data
            if (data.battery_voltage !== undefined) {
                stateDisplay += `🔋 Battery: ${data.battery_voltage.toFixed(1)}V`;
                if (data.battery_percentage !== undefined) {
                    stateDisplay += ` (${data.battery_percentage.toFixed(0)}%)`;
                }
                stateDisplay += '<br>';
            }
            
            if (data.system_temperature !== undefined) {
                stateDisplay += `🌡️ Temperature: ${data.system_temperature.toFixed(1)}°C<br>`;
            }
            
            if (data.current_mode !== undefined) {
                stateDisplay += `⚙️ Mode: ${data.current_mode}<br>`;
            }
            
            if (data.emergency_stopped !== undefined) {
                const status = data.emergency_stopped ? '🛑 E-STOP ACTIVE' : '✅ Normal';
                stateDisplay += `Status: ${status}<br>`;
            }
            
            if (data.position) {
                stateDisplay += `📍 Position: (${data.position.x.toFixed(2)}, ${data.position.y.toFixed(2)}, ${data.position.z.toFixed(2)})<br>`;
            }
        } else {
            stateDisplay = data.toString();
        }
        
        this.elements.robotState.innerHTML = `
            <div style="color: #4CAF50; font-weight: bold;">
                ${new Date().toLocaleTimeString()}<br>
                ${stateDisplay}
            </div>
        `;
    }

    handleDiagnostics(data) {
        const levelColors = {
            0: '#4CAF50', // OK
            1: '#ff9800', // WARN
            2: '#f44336', // ERROR
            3: '#9e9e9e'  // STALE
        };
        
        const levelNames = ['OK', 'WARN', 'ERROR', 'STALE'];
        const color = levelColors[data.level] || '#333';
        const levelName = levelNames[data.level] || 'UNKNOWN';
        
        this.elements.diagnostics.innerHTML = `
            <div style="color: ${color}; font-weight: bold;">
                ${new Date().toLocaleTimeString()}<br>
                <strong>${data.name}</strong> [${levelName}]<br>
                ${data.message}<br>
                ${data.hardware_id ? `Hardware: ${data.hardware_id}` : ''}
            </div>
        `;
    }

    updateConnectionStatus(status) {
        this.elements.connectionStatus.className = `status ${status}`;
        
        switch (status) {
            case 'connected':
                this.elements.connectionStatus.textContent = 'Connected';
                this.elements.connectBtn.textContent = 'Disconnect';
                this.elements.connectBtn.className = 'btn secondary';
                this.elements.configureBtn.disabled = false;
                this.elements.testBtn.disabled = false;
                break;
            case 'connecting':
                this.elements.connectionStatus.textContent = 'Connecting';
                this.elements.connectBtn.textContent = 'Connecting...';
                this.elements.connectBtn.disabled = true;
                this.elements.configureBtn.disabled = true;
                this.elements.testBtn.disabled = true;
                break;
            case 'disconnected':
            default:
                this.elements.connectionStatus.textContent = 'Disconnected';
                this.elements.connectBtn.textContent = 'Connect';
                this.elements.connectBtn.className = 'btn primary';
                this.elements.connectBtn.disabled = false;
                this.elements.configureBtn.disabled = true;
                this.elements.testBtn.disabled = true;
                break;
        }
    }

    setupTopics() {
        // Subscribe to CAN messages
        this.canTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/can/raw',
            messageType: 'can_msgs/msg/Frame'
        });

        this.canTopic.subscribe((message) => {
            this.handleCanMessage(message);
        });

        // Subscribe to robot state (if available)
        this.robotStateTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/robot_state',
            messageType: 'std_msgs/msg/String'
        });

        this.robotStateTopic.subscribe((message) => {
            this.handleRobotState(message);
        });

        this.log('Subscribed to robot topics', 'success');
    }

    handleCanMessage(message) {
        this.canMessageCount++;
        this.canRateCounter++;
        this.elements.canCount.textContent = this.canMessageCount;

        // Convert data array to hex string
        const dataHex = Array.from(message.data)
            .map(b => b.toString(16).padStart(2, '0').toUpperCase())
            .join('');

        // Create message element
        const messageElement = document.createElement('div');
        messageElement.className = 'can-message';
        messageElement.innerHTML = `
            <div class="timestamp">${new Date().toLocaleTimeString()}</div>
            <div>ID: <span class="id">0x${message.id.toString(16).toUpperCase()}</span></div>
            <div>Data: <span class="data">${dataHex}</span></div>
        `;

        // Add to display (keep only last 20 messages)
        const container = this.elements.canMessages;
        if (container.children.length >= 20) {
            container.removeChild(container.firstChild);
        }
        
        // Remove "no data" message if present
        const noDataMsg = container.querySelector('.no-data');
        if (noDataMsg) {
            noDataMsg.remove();
        }

        container.appendChild(messageElement);
        container.scrollTop = container.scrollHeight;
    }

    handleRobotState(message) {
        this.elements.robotState.innerHTML = `
            <div style="color: #4CAF50; font-weight: bold;">
                ${new Date().toLocaleTimeString()}: ${message.data}
            </div>
        `;
    }

    configureGateway() {
        this.log('Configuring CAN gateway...', 'info');

        // Configure the gateway
        const configureService = new ROSLIB.Service({
            ros: this.ros,
            name: '/can_gateway/change_state',
            serviceType: 'lifecycle_msgs/srv/ChangeState'
        });

        const configureRequest = new ROSLIB.ServiceRequest({
            transition: { id: 1 }
        });

        configureService.callService(configureRequest, (result) => {
            this.log('Gateway configured successfully', 'success');
            
            // Now activate it
            const activateRequest = new ROSLIB.ServiceRequest({
                transition: { id: 3 }
            });

            configureService.callService(activateRequest, (result) => {
                this.log('Gateway activated successfully', 'success');
            }, (error) => {
                this.log(`Failed to activate gateway: ${error}`, 'error');
            });
        }, (error) => {
            this.log(`Failed to configure gateway: ${error}`, 'error');
        });
    }

    testGateway() {
        this.log('Testing CAN gateway...', 'info');

        // Check if we can see topics
        this.ros.getTopics((topics) => {
            const canTopics = topics.topics.filter(topic => topic.includes('can'));
            if (canTopics.length > 0) {
                this.log(`Found CAN topics: ${canTopics.join(', ')}`, 'success');
            } else {
                this.log('No CAN topics found', 'warning');
            }
        });

        // Send a test message (this would require a publisher)
        this.log('Test completed - check CAN message display above', 'info');
    }

    log(message, type = 'info') {
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry ${type}`;
        logEntry.innerHTML = `<span class="timestamp">[${timestamp}]</span> ${message}`;
        
        this.elements.activityLog.appendChild(logEntry);
        this.elements.activityLog.scrollTop = this.elements.activityLog.scrollHeight;

        // Keep only last 100 log entries
        while (this.elements.activityLog.children.length > 100) {
            this.elements.activityLog.removeChild(this.elements.activityLog.firstChild);
        }
    }

    clearLog() {
        this.elements.activityLog.innerHTML = '';
        this.log('Activity log cleared', 'info');
    }
}

// Initialize the app when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    window.evabotApp = new EvabotApp();
});

// Handle PWA install prompt
let deferredPrompt;
window.addEventListener('beforeinstallprompt', (e) => {
    deferredPrompt = e;
    // Show install button or notification
    console.log('PWA install prompt available');
});

window.addEventListener('appinstalled', () => {
    console.log('PWA was installed');
});

// Handle network status for offline capabilities
window.addEventListener('online', () => {
    console.log('Back online');
    if (window.evabotApp && !window.evabotApp.connected) {
        setTimeout(() => window.evabotApp.connect(), 1000);
    }
});

window.addEventListener('offline', () => {
    console.log('Gone offline');
});
