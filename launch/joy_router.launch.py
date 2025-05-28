#!/usr/bin/env python3
"""
Launch file for the joy_msg_router node.

This launch file starts the joy_msg_router node with configurable parameters.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """Generate the launch description for joy_msg_router."""
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to configuration file. If empty, uses default hardcoded config.'
    )
    
    profile_arg = DeclareLaunchArgument(
        'profile',
        default_value='teleop',
        description='Profile to use from configuration file (default: teleop)'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )
    
    joy_topic_arg = DeclareLaunchArgument(
        'joy_topic',
        default_value='/joy',
        description='Input joy topic name'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Output twist topic name'
    )
    
    use_default_config_arg = DeclareLaunchArgument(
        'use_default_config',
        default_value='true',
        description='Whether to use the default config file from the package'
    )
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    profile = LaunchConfiguration('profile')
    namespace = LaunchConfiguration('namespace')
    joy_topic = LaunchConfiguration('joy_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_default_config = LaunchConfiguration('use_default_config')
    
    # Path to default config file
    default_config_file = PathJoinSubstitution([
        FindPackageShare('joy_msg_router_rs'),
        'config',
        'default.yaml'
    ])
    
    # Joy message router node with default config
    joy_router_node_with_config = Node(
        package='joy_msg_router_rs',
        executable='joy_msg_router',
        name='joy_msg_router',
        namespace=namespace,
        arguments=['--config', default_config_file, '--profile', profile],
        remappings=[
            ('joy', joy_topic),
            ('cmd_vel', cmd_vel_topic),
        ],
        output='screen',
        condition=IfCondition(use_default_config)
    )
    
    # Joy message router node with custom config
    joy_router_node_custom = Node(
        package='joy_msg_router_rs',
        executable='joy_msg_router',
        name='joy_msg_router',
        namespace=namespace,
        arguments=['--config', config_file, '--profile', profile],
        remappings=[
            ('joy', joy_topic),
            ('cmd_vel', cmd_vel_topic),
        ],
        output='screen',
        condition=UnlessCondition(use_default_config)
    )
    
    # Log information about the launch
    log_config_info = LogInfo(
        msg=[
            'Starting joy_msg_router with profile: ', profile,
            ', joy topic: ', joy_topic,
            ', cmd_vel topic: ', cmd_vel_topic
        ]
    )
    
    return LaunchDescription([
        # Declare arguments
        config_file_arg,
        profile_arg,
        namespace_arg,
        joy_topic_arg,
        cmd_vel_topic_arg,
        use_default_config_arg,
        
        # Log launch info
        log_config_info,
        
        # Launch nodes
        joy_router_node_with_config,
        joy_router_node_custom,
    ])