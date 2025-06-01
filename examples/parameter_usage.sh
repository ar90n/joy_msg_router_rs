#!/bin/bash
# Examples of using joy_msg_router with different parameter configurations

echo "Joy Message Router - Usage Examples"
echo "==================================="

# Example 1: Using hierarchical YAML config file
echo ""
echo "1. Using hierarchical YAML configuration file (recommended):"
echo "   ros2 run joy_msg_router_rs joy_msg_router --ros-args -p config_file:=/path/to/config.yaml"
echo ""

# Example 2: Selecting a specific profile from config file
echo "2. Selecting a specific profile from config file:"
echo "   ros2 run joy_msg_router_rs joy_msg_router --ros-args \\"
echo "     -p config_file:=config.yaml -p profile_name:=teleop_with_modifiers"
echo ""

# Example 3: Using ROS2 parameters directly
echo "3. Using ROS2 parameters directly (flat format):"
echo "   ros2 run joy_msg_router_rs joy_msg_router --ros-args \\"
echo "     -p profile_name:=custom \\"
echo "     -p input_mappings.0.source_type:=axis \\"
echo "     -p input_mappings.0.source_index:=1 \\"
echo "     -p input_mappings.0.action_type:=publish \\"
echo "     -p input_mappings.0.topic:=/cmd_vel \\"
echo "     -p input_mappings.0.message_type:=\"geometry_msgs/msg/Twist\" \\"
echo "     -p input_mappings.0.field:=\"linear.x\" \\"
echo "     -p input_mappings.0.scale:=0.5"
echo ""

# Example 4: Launch files with config
echo "4. Using launch files with config file:"
echo "   ros2 launch joy_msg_router_rs joy_router.launch.py \\"
echo "     config_file:=./config/default.yaml profile_name:=teleop_safe"
echo ""

# Example 5: Multi-robot setup
echo "5. Multi-robot setup with different configurations:"
echo "   # Terminal 1 - Robot with turbo mode"
echo "   ros2 launch joy_msg_router_rs joy_teleop.launch.py \\"
echo "     namespace:=robot1 config_file:=config.yaml profile_name:=teleop_with_modifiers \\"
echo "     joy_topic:=/robot1/joy cmd_vel_topic:=/robot1/cmd_vel"
echo ""
echo "   # Terminal 2 - Robot with safe mode"
echo "   ros2 launch joy_msg_router_rs joy_teleop.launch.py \\"
echo "     namespace:=robot2 config_file:=config.yaml profile_name:=teleop_safe \\"
echo "     joy_topic:=/robot2/joy cmd_vel_topic:=/robot2/cmd_vel"
echo ""

# Available profiles
echo "Available profiles in default.yaml:"
echo "  - teleop:                Basic teleoperation"
echo "  - teleop_safe:           With deadman switch (L1 button)"
echo "  - teleop_with_modifiers: With turbo, precision, and emergency stop"
echo "  - multi_output:          Examples of different message types"
echo ""

# Modifier examples
echo "Modifier functionality examples:"
echo "  - Turbo mode:     Hold R1 to double speed"
echo "  - Precision mode: Hold L1 for 30% speed"
echo "  - Variable boost: Use right trigger for gradual speed increase"
echo "  - Emergency stop: Press X to zero all movement"