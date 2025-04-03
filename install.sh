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

# Prompt for ROS_DOMAIN_ID
read -p "Enter ROS_DOMAIN_ID for network communication (0-232, default: 42): " ros_domain_id
ros_domain_id=${ros_domain_id:-42}

# Create required directories
echo "Creating required directories..."
mkdir -p $HOME_DIR/test
mkdir -p $HOME_DIR/gcode
chown -R $RUN_USER:$RUN_USER $HOME_DIR/test $HOME_DIR/gcode
chmod -R 755 $HOME_DIR/test $HOME_DIR/gcode

# Create starter script that takes component name as argument
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

def main():
    # Get component from command line argument
    if len(sys.argv) < 2 or sys.argv[1] not in COMPONENTS:
        print(f"Usage: {sys.argv[0]} [main|gantry|printer]")
        sys.exit(1)
    
    component = sys.argv[1]
    component_cmd = COMPONENTS[component]
    
    # Run the commands
    print(f"Starting mc2425 {component} component...")
    os.system(component_cmd)

if __name__ == "__main__":
    main()
EOF

chmod +x /usr/local/bin/mc2425_starter.py

# Function to install a component service
install_component() {
    local component=$1
    local printer_id=$2
    local service_name="mc2425-${component}"
    
    echo "Installing $service_name service..."
    
    # Create specific environment variables
    local env_vars="Environment=\"MC2425_SYSTEM_TYPE=$component\""
    if [ "$component" == "printer" ] && [ -n "$printer_id" ]; then
        env_vars="$env_vars\nEnvironment=\"PRINTER_ID=$printer_id\""
    fi
    
    # Create ROS startup script with proper environment sourcing
    local ros_starter="/usr/local/bin/start_${component}.sh"
    cat > $ros_starter << EOF
#!/bin/bash
# ROS2 startup script for $component

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source $HOME_DIR/Workcell-ROS/install/setup.bash

# Set ROS2 network configuration
export ROS_DOMAIN_ID=$ros_domain_id
export ROS_LOCALHOST_ONLY=0

# Execute the component
exec ros2 run mc2425 $component
EOF

    chmod +x $ros_starter
    
    # Create systemd service
    cat > /etc/systemd/system/$service_name.service << EOF
[Unit]
Description=MC2425 ROS2 $component Component
After=network.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$HOME_DIR
ExecStart=$ros_starter
Restart=on-failure
RestartSec=5
$(echo -e $env_vars)

[Install]
WantedBy=multi-user.target
EOF

    # Enable and start the service
    systemctl daemon-reload
    systemctl enable $service_name.service
    systemctl restart $service_name.service
    
    echo "$service_name service has been installed and started!"
}

# Multi-select menu for components
echo "Select which components to run on this machine (enter numbers separated by space):"
echo "1) Main Controller"
echo "2) Gantry"
echo "3) Printer"
read -p "Enter choices [e.g. 1 2 3]: " choices

for choice in $choices; do
    case $choice in
        1) 
            install_component "main" ""
            ;;
        2) 
            install_component "gantry" ""
            ;;
        3) 
            read -p "Enter printer ID: " printer_id
            install_component "printer" "$printer_id"
            ;;
        *) 
            echo "Invalid choice: $choice (skipping)"
            ;;
    esac
done

echo ""
echo "Installation complete!"
echo "Important notes:"
echo "- Directories $HOME_DIR/test and $HOME_DIR/gcode have been created"
echo "- Services are set to run as user: $RUN_USER"
echo "- The working directory is set to: $HOME_DIR"
echo "- ROS_DOMAIN_ID is set to: $ros_domain_id (ensure all machines use the same value)"
echo "- ROS_LOCALHOST_ONLY is set to 0 (enabling network communication)"
echo "- Check status of services with: sudo systemctl status mc2425-*.service"
echo "- View logs with: sudo journalctl -u mc2425-main.service (replace 'main' with component name)"
echo ""
echo "If nodes can't communicate between machines, check:"
echo "1. Ensure both machines can ping each other"
echo "2. Verify firewalls allow ROS2 communication"
echo "3. Make sure ROS_DOMAIN_ID is identical on all machines"
