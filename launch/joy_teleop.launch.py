#!/usr/bin/env python3
"""
Complete teleoperation launch file.

This launch file starts both the joy node (for reading joystick input)
and the joy_msg_router node for processing the joystick commands.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the complete teleoperation launch description."""
    
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device file'
    )
    
    profile_arg = DeclareLaunchArgument(
        'profile',
        default_value='teleop',
        description='Joy router profile to use (teleop, teleop_safe, holonomic, drone)'
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
    
    # Get launch configurations
    device = LaunchConfiguration('device')
    profile = LaunchConfiguration('profile')
    namespace = LaunchConfiguration('namespace')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    deadzone = LaunchConfiguration('deadzone')
    autorepeat_rate = LaunchConfiguration('autorepeat_rate')
    
    # Path to default config file
    config_file = PathJoinSubstitution([
        FindPackageShare('joy_msg_router_rs'),
        'config',
        'default.yaml'
    ])
    
    # Joy node for reading joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            {'device': device},
            {'deadzone': deadzone},
            {'autorepeat_rate': autorepeat_rate},
        ],
        output='screen'
    )
    
    # Joy message router node
    joy_router_node = Node(
        package='joy_msg_router_rs',
        executable='joy_msg_router',
        name='joy_msg_router',
        arguments=['--config', config_file, '--profile', profile],
        remappings=[
            ('cmd_vel', cmd_vel_topic),
        ],
        output='screen'
    )
    
    # Group all nodes under namespace if provided
    teleop_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            joy_node,
            joy_router_node,
        ]
    )
    
    # Log launch information
    launch_info = LogInfo(
        msg=[
            'Starting teleoperation with device: ', device,
            ', profile: ', profile,
            ', namespace: ', namespace,
            ', cmd_vel topic: ', cmd_vel_topic
        ]
    )
    
    return LaunchDescription([
        # Declare arguments
        device_arg,
        profile_arg,
        namespace_arg,
        cmd_vel_topic_arg,
        deadzone_arg,
        autorepeat_rate_arg,
        
        # Log info
        launch_info,
        
        # Launch group
        teleop_group,
    ])