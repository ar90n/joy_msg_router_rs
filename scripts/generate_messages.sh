#!/bin/bash
# Script to generate ROS2 messages for safe_drive

set -e

# Source ROS2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "Error: ROS2 humble not found!"
        exit 1
    fi
fi

echo "Generating ROS2 messages for safe_drive..."
echo "ROS_DISTRO: $ROS_DISTRO"
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"

# Install safe_drive_msg if not already installed
if ! command -v safe_drive_msg &> /dev/null; then
    echo "Installing safe_drive_msg..."
    cargo install safe_drive_msg --version 0.2
fi

# Create message directory
rm -rf /tmp/safe_drive_msgs
mkdir -p /tmp/safe_drive_msgs

# Generate messages with error handling
generate_msg() {
    local pkg=$1
    echo "Generating $pkg..."
    if safe_drive_msg $pkg -o /tmp/safe_drive_msgs -p; then
        echo "✓ $pkg generated successfully"
    else
        echo "✗ Failed to generate $pkg"
        return 1
    fi
}

# Generate all required messages
generate_msg std_msgs
generate_msg sensor_msgs
generate_msg geometry_msgs

echo "Message generation complete!"
echo "Generated files:"
ls -la /tmp/safe_drive_msgs/