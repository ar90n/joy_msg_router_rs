#!/bin/bash
# Examples of using joy_msg_router with different parameter configurations

echo "Joy Message Router - Parameter Usage Examples"
echo "============================================="

# Example 1: Default configuration
echo ""
echo "1. Running with default configuration:"
echo "   ros2 run joy_msg_router_rs joy_msg_router"
echo ""

# Example 2: Command line profile selection
echo "2. Selecting a specific profile via command line:"
echo "   ros2 run joy_msg_router_rs joy_msg_router --profile drone"
echo ""

# Example 3: Custom configuration file
echo "3. Using a custom configuration file:"
echo "   ros2 run joy_msg_router_rs joy_msg_router --config /path/to/config.yaml --profile my_robot"
echo ""

# Example 4: Environment variables
echo "4. Using environment variables:"
echo "   export JOY_ROUTER_CONFIG_FILE=/path/to/config.yaml"
echo "   export JOY_ROUTER_PROFILE=holonomic"
echo "   ros2 run joy_msg_router_rs joy_msg_router"
echo ""

# Example 5: Launch files with parameters
echo "5. Using launch files with parameters:"
echo "   ros2 launch joy_msg_router_rs joy_router.launch.py profile:=teleop_safe"
echo "   ros2 launch joy_msg_router_rs joy_teleop.launch.py profile:=drone device:=/dev/input/js1"
echo ""

# Example 6: Multi-robot with different profiles
echo "6. Multi-robot setup with different profiles:"
echo "   # Terminal 1 - Differential drive robot"
echo "   ros2 launch joy_msg_router_rs joy_teleop.launch.py \\"
echo "     namespace:=robot1 profile:=teleop cmd_vel_topic:=/robot1/cmd_vel"
echo ""
echo "   # Terminal 2 - Holonomic robot"  
echo "   ros2 launch joy_msg_router_rs joy_teleop.launch.py \\"
echo "     namespace:=robot2 profile:=holonomic cmd_vel_topic:=/robot2/cmd_vel device:=/dev/input/js1"
echo ""

# Parameter priority explanation
echo "Parameter Priority (highest to lowest):"
echo "  1. Command line arguments (--config, --profile)"
echo "  2. Environment variables (JOY_ROUTER_CONFIG_FILE, JOY_ROUTER_PROFILE)"
echo "  3. Default values (config/default.yaml, teleop profile)"
echo ""

echo "Available profiles in default configuration:"
echo "  - teleop:      Basic differential drive robot"
echo "  - teleop_safe: Teleop with deadman switch (L1 button)"
echo "  - holonomic:   Omnidirectional robot with strafe"
echo "  - drone:       UAV/drone with 3D movement"