#!/usr/bin/env python3
"""
Simple test to verify the joy_msg_router node can be launched.
"""

import os
import sys
import time
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def test_node_launches():
    """Test that the node can be launched successfully."""
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create a test node
        test_node = Node("test_launcher")
        
        # Start the joy_msg_router in a subprocess
        env = os.environ.copy()
        # Make sure the node is in the PATH
        if 'AMENT_PREFIX_PATH' in env:
            # Add install directories to PATH
            install_dirs = env['AMENT_PREFIX_PATH'].split(':')
            path_additions = [os.path.join(d, 'lib', 'joy_msg_router_rs') for d in install_dirs]
            env['PATH'] = ':'.join(path_additions) + ':' + env.get('PATH', '')
        
        process = subprocess.Popen(
            ["joy_msg_router"],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        # Give it time to start
        time.sleep(2)
        
        # Check if process is still running
        poll_result = process.poll()
        if poll_result is not None:
            stdout, stderr = process.communicate()
            print(f"Process exited with code {poll_result}")
            print(f"STDOUT: {stdout.decode()}")
            print(f"STDERR: {stderr.decode()}")
            assert False, f"Node exited prematurely with code {poll_result}"
        
        # Check that topics exist
        topic_list = test_node.get_topic_names_and_types()
        topic_names = [name for name, _ in topic_list]
        
        assert '/joy' in topic_names, "/joy topic not found"
        assert '/cmd_vel' in topic_names, "/cmd_vel topic not found"
        
        print("✓ Node launched successfully")
        print("✓ Expected topics are present")
        
        # Cleanup
        process.terminate()
        process.wait(timeout=5)
        test_node.destroy_node()
        
    finally:
        rclpy.shutdown()


def test_message_routing():
    """Test basic message routing functionality."""
    rclpy.init()
    
    try:
        test_node = Node("test_routing")
        
        # Start the node
        env = os.environ.copy()
        process = subprocess.Popen(
            ["ros2", "run", "joy_msg_router_rs", "joy_msg_router"],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        time.sleep(2)
        
        # Set up publisher and subscriber
        joy_pub = test_node.create_publisher(Joy, "/joy", 10)
        received_twists = []
        
        def twist_callback(msg):
            received_twists.append(msg)
        
        twist_sub = test_node.create_subscription(
            Twist, "/cmd_vel", twist_callback, 10
        )
        
        # Wait for connections
        time.sleep(1)
        
        # Send a joy message
        joy_msg = Joy()
        joy_msg.axes = [0.0, 0.5, 0.0, 0.3, 0.0, 0.0]
        joy_msg.buttons = []
        
        # Publish multiple times
        for i in range(10):
            joy_pub.publish(joy_msg)
            rclpy.spin_once(test_node, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Check results
        assert len(received_twists) > 0, "No Twist messages received"
        
        # With default config:
        # axis 1 -> linear.x with scale 0.5
        # axis 3 -> angular.z with scale 1.0
        last_twist = received_twists[-1]
        expected_linear_x = 0.5 * 0.5  # 0.25
        expected_angular_z = 0.3 * 1.0  # 0.3
        
        assert abs(last_twist.linear.x - expected_linear_x) < 0.01, \
            f"Expected linear.x={expected_linear_x}, got {last_twist.linear.x}"
        assert abs(last_twist.angular.z - expected_angular_z) < 0.01, \
            f"Expected angular.z={expected_angular_z}, got {last_twist.angular.z}"
        
        print("✓ Message routing works correctly")
        print(f"✓ Received {len(received_twists)} Twist messages")
        
        # Cleanup
        process.terminate()
        process.wait(timeout=5)
        test_node.destroy_node()
        
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    print("Testing joy_msg_router node...")
    print("-" * 50)
    
    print("\n1. Testing node launch...")
    test_node_launches()
    
    print("\n2. Testing message routing...")
    test_message_routing()
    
    print("\n" + "="*50)
    print("All tests passed! ✓")