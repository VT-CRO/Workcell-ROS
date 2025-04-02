#!/bin/bash
# Installation script for MC2425 ROS2 autostart

# Verify root privileges
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run as root" 
    exit 1
fi

# Get the username from SUDO_USER if available, otherwise use current user
if [ -n "$SUDO_USER" ]; then
    RUN_USER=$SUDO_USER
else
    RUN_USER=$(whoami)
fi

HOME_DIR=$(eval echo ~$RUN_USER)
echo "Using home directory: $HOME_DIR"

# Prompt for system type
echo "Which MC2425 component would you like to run on startup?"
echo "1) Main Controller"
echo "2) Gantry"
echo "3) Printer"
read -p "Enter choice [1-3]: " choice

case $choice in
    1) 
        SYSTEM_TYPE="main"
        ENV_VARS="Environment=\"MC2425_SYSTEM_TYPE=main\""
        ;;
    2) 
        SYSTEM_TYPE="gantry"
        ENV_VARS="Environment=\"MC2425_SYSTEM_TYPE=gantry\""
        ;;
    3) 
        SYSTEM_TYPE="printer"
        read -p "Enter printer ID: " printer_id
        ENV_VARS="Environment=\"MC2425_SYSTEM_TYPE=printer\"\nEnvironment=\"PRINTER_ID=$printer_id\""
        ;;
    *) 
        echo "Invalid choice"
        exit 1 
        ;;
esac

# Create required directories
echo "Creating required directories..."
mkdir -p $HOME_DIR/test
mkdir -p $HOME_DIR/gcode
chown -R $RUN_USER:$RUN_USER $HOME_DIR/test $HOME_DIR/gcode
chmod -R 755 $HOME_DIR/test $HOME_DIR/gcode

# Create starter script
cat > /usr/local/bin/mc2425_starter.py << 'EOF'
#!/usr/bin/env python3
import os
import subprocess
import sys

# Define possible components to run
COMPONENTS = {
    "main": "ros2 run mc2425 mainController",
    "gantry": "ros2 run mc2425 gantry",
    "printer": "ros2 run mc2425 printer"
}

def determine_system_type():
    """
    Determine which system is running based on environment variables or hostname
    """
    # Check for environment variable first
    system_type = os.environ.get("MC2425_SYSTEM_TYPE")
    if system_type and system_type in COMPONENTS:
        return system_type
        
    # Default to main if cannot determine
    return "main"

def main():
    # Source ROS environment
    ros_setup = "source /opt/ros/jazzy/setup.bash"
    
    # Find path to your workspace setup
    # Using find command to locate setup.bash in home directory
    workspace_setup = "source ~/Workcell-ROS/install/setup.bash"
    
    # Determine which component to run
    system_type = determine_system_type()
    component_cmd = COMPONENTS[system_type]
    
    # Run the commands
    full_cmd = f"{ros_setup} && {workspace_setup} && {component_cmd}"
    print(f"Starting mc2425 {system_type} component...")
    os.system(f"bash -c '{full_cmd}'")

if __name__ == "__main__":
    main()
EOF

chmod +x /usr/local/bin/mc2425_starter.py

# Create systemd service
cat > /etc/systemd/system/mc2425.service << EOF
[Unit]
Description=MC2425 ROS2 System
After=network.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$HOME_DIR
ExecStart=/usr/local/bin/mc2425_starter.py
Restart=on-failure
RestartSec=5
$(echo -e $ENV_VARS)

[Install]
WantedBy=multi-user.target
EOF

# Enable and start the service
systemctl daemon-reload
systemctl enable mc2425.service
systemctl restart mc2425.service

echo "MC2425 $SYSTEM_TYPE has been set to start on boot!"
echo "Check status with: sudo systemctl status mc2425.service"
echo ""
echo "Important notes:"
echo "- Directories $HOME_DIR/test and $HOME_DIR/gcode have been created"
echo "- The service is set to run as user: $RUN_USER"
echo "- The working directory is set to: $HOME_DIR"
echo "- Restart with: sudo systemctl restart mc2425.service"
echo "- View logs with: sudo journalctl -u mc2425.service"