#!/usr/bin/env python3
"""
Complete teleoperation launch file.

This launch file starts both the joy node (for reading joystick input)
and the joy_msg_router node for processing the joystick commands.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Setup function to handle parameter resolution for joy_msg_router."""
    namespace = LaunchConfiguration('namespace').perform(context)
    joy_topic = LaunchConfiguration('joy_topic').perform(context)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)
    param_file = LaunchConfiguration('param_file').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    profile_name = LaunchConfiguration('profile_name').perform(context)
    device = LaunchConfiguration('device').perform(context)
    deadzone = LaunchConfiguration('deadzone').perform(context)
    autorepeat_rate = LaunchConfiguration('autorepeat_rate').perform(context)
    
    # Prepare parameters for joy_msg_router
    parameters = []
    
    # If config_file is specified, add it as a parameter
    if config_file:
        if not os.path.isabs(config_file):
            config_file = os.path.abspath(config_file)
        parameters.append({'config_file': config_file})
        if profile_name:
            parameters.append({'profile_name': profile_name})
    elif param_file:
        # Convert relative path to absolute if needed
        if not os.path.isabs(param_file):
            param_file = os.path.abspath(param_file)
        parameters.append(param_file)
    
    # Joy node for reading joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=namespace,
        parameters=[
            {'device': device},
            {'deadzone': float(deadzone)},
            {'autorepeat_rate': float(autorepeat_rate)},
        ],
        output='screen'
    )
    
    # Joy message router node
    joy_router_node = Node(
        package='joy_msg_router_rs',
        executable='joy_msg_router',
        name='joy_msg_router',
        namespace=namespace,
        parameters=parameters,
        remappings=[
            ('joy', joy_topic),
            ('cmd_vel', cmd_vel_topic),
        ],
        output='screen'
    )
    
    return [joy_node, joy_router_node]


def generate_launch_description():
    """Generate the complete teleoperation launch description."""
    
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device file'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Output topic for velocity commands'
    )
    
    deadzone_arg = DeclareLaunchArgument(
        'deadzone',
        default_value='0.1',
        description='Deadzone for joystick axes'
    )
    
    autorepeat_rate_arg = DeclareLaunchArgument(
        'autorepeat_rate',
        default_value='20.0',
        description='Rate for auto-repeating joystick messages (Hz)'
    )
    
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('joy_msg_router_rs'),
            'config',
            'default_params.yaml'
        ]),
        description='Path to ROS parameter file (YAML) for joy_msg_router. If empty, parameters must be set separately.'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to hierarchical YAML configuration file. Takes precedence over param_file.'
    )
    
    profile_name_arg = DeclareLaunchArgument(
        'profile_name',
        default_value='',
        description='Profile name to use when loading from config_file.'
    )
    
    joy_topic_arg = DeclareLaunchArgument(
        'joy_topic',
        default_value='/joy',
        description='Joy topic name'
    )
    
    # Use OpaqueFunction to handle parameter resolution
    launch_setup_function = OpaqueFunction(function=launch_setup)
    
    return LaunchDescription([
        # Declare arguments
        device_arg,
        namespace_arg,
        cmd_vel_topic_arg,
        deadzone_arg,
        autorepeat_rate_arg,
        param_file_arg,
        config_file_arg,
        profile_name_arg,
        joy_topic_arg,
        
        # Launch nodes using OpaqueFunction
        launch_setup_function,
    ])