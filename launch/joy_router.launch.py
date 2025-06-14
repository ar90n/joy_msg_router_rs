#!/usr/bin/env python3
"""
Launch file for the joy_msg_router node.

This launch file starts the joy_msg_router node with ROS parameters.
Since the node now uses ROS parameters instead of file-based configuration,
parameters should be loaded using a parameter file or set directly.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Setup function to handle parameter file path resolution."""
    namespace = LaunchConfiguration('namespace').perform(context)
    joy_topic = LaunchConfiguration('joy_topic').perform(context)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)
    param_file = LaunchConfiguration('param_file').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    profile_name = LaunchConfiguration('profile_name').perform(context)
    
    # Prepare parameters
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
    
    return [joy_router_node]


def generate_launch_description():
    """Generate the launch description for joy_msg_router."""
    
    # Declare launch arguments
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
    
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('joy_msg_router_rs'),
            'config',
            'default_params.yaml'
        ]),
        description='Path to ROS parameter file (YAML). If empty, parameters must be set separately.'
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
    
    # Get launch configurations
    joy_topic = LaunchConfiguration('joy_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    
    # Log information about the launch
    log_info = LogInfo(
        msg=[
            'Starting joy_msg_router with joy topic: ', joy_topic,
            ', cmd_vel topic: ', cmd_vel_topic
        ]
    )
    
    return LaunchDescription([
        # Declare arguments
        namespace_arg,
        joy_topic_arg,
        cmd_vel_topic_arg,
        param_file_arg,
        config_file_arg,
        profile_name_arg,
        
        # Log launch info
        log_info,
        
        # Launch node using OpaqueFunction
        OpaqueFunction(function=launch_setup),
    ])