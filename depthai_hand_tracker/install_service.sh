#!/bin/bash

# Robot Hand Auto-Start Installation Script
# This script installs the robot hand control as a systemd service

echo "=========================================="
echo "Robot Hand Auto-Start Installer"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "❌ Please run with sudo:"
    echo "   sudo bash install_service.sh"
    exit 1
fi

# Get the actual user (not root)
ACTUAL_USER=${SUDO_USER:-$USER}
USER_HOME=$(eval echo ~$ACTUAL_USER)

echo "Installing for user: $ACTUAL_USER"
echo "Home directory: $USER_HOME"
echo ""

# Determine the current directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "Project directory: $PROJECT_DIR"
echo ""

# Create a temporary service file with correct paths
SERVICE_FILE="/tmp/robot_hand.service"
cat > $SERVICE_FILE << EOF
[Unit]
Description=Robot Hand Control Service
After=graphical.target

[Service]
Type=simple
User=$ACTUAL_USER
WorkingDirectory=$SCRIPT_DIR
ExecStart=/usr/bin/lxterminal --title="Robot Hand Control" -e "$SCRIPT_DIR/.venv/bin/python3 $SCRIPT_DIR/robot_hand.py"
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

# Environment variables
Environment="DISPLAY=:0"
Environment="XAUTHORITY=$USER_HOME/.Xauthority"

[Install]
WantedBy=graphical.target
EOF

echo "✓ Service file created"

# Copy service file to systemd directory
cp $SERVICE_FILE /etc/systemd/system/robot_hand.service
echo "✓ Service file installed to /etc/systemd/system/"

# Reload systemd daemon
systemctl daemon-reload
echo "✓ Systemd daemon reloaded"

# Enable the service
systemctl enable robot_hand.service
echo "✓ Service enabled (will start on boot)"

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "Available commands:"
echo "  sudo systemctl start robot_hand      - Start the service now"
echo "  sudo systemctl stop robot_hand       - Stop the service"
echo "  sudo systemctl restart robot_hand    - Restart the service"
echo "  sudo systemctl status robot_hand     - Check service status"
echo "  sudo systemctl disable robot_hand    - Disable auto-start"
echo "  sudo journalctl -u robot_hand -f     - View live logs"
echo ""
echo "To start the service now, run:"
echo "  sudo systemctl start robot_hand"
echo ""
