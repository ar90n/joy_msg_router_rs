#!/usr/bin/env python3
"""
Node-level tests for the joy_msg_router node.

These tests verify the ROS2 integration and message routing functionality.
"""

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time
import threading
import subprocess
import os


class TestJoyRouter:
    """Test class for joy_msg_router node testing."""
    
    @classmethod
    def setup_class(cls):
        """Initialize ROS2 for testing."""
        rclpy.init()
    
    @classmethod
    def teardown_class(cls):
        """Shutdown ROS2 after testing."""
        rclpy.shutdown()
    
    def test_node_starts_and_publishes(self):
        """Test that the node starts and publishes Twist messages when receiving Joy messages."""
        test_node = Node("test_joy_router")
        
        # Create publishers and subscribers
        joy_pub = test_node.create_publisher(Joy, "/joy", 10)
        received_twists = []
        
        def twist_callback(msg):
            received_twists.append(msg)
        
        twist_sub = test_node.create_subscription(
            Twist, "/cmd_vel", twist_callback, 10
        )
        
        # Start the joy_msg_router node in a subprocess
        env = os.environ.copy()
        process = subprocess.Popen(
            ["ros2", "run", "joy_msg_router_rs", "joy_msg_router"],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        try:
            # Give the node time to start
            time.sleep(2)
            
            # Create and publish a Joy message
            joy_msg = Joy()
            joy_msg.axes = [0.0, 0.5, 0.0, 0.3]  # Forward and turn
            joy_msg.buttons = [0, 0, 0, 0, 0]
            
            # Publish several times to ensure message is received
            for _ in range(5):
                joy_pub.publish(joy_msg)
                rclpy.spin_once(test_node, timeout_sec=0.1)
                time.sleep(0.1)
            
            # Check that we received Twist messages
            assert len(received_twists) > 0, "No Twist messages received"
            
            # Verify the twist values (using default hardcoded config)
            # Default config maps axis 1 to linear.x with scale 0.5
            # and axis 3 to angular.z with scale 1.0
            twist = received_twists[-1]
            assert abs(twist.linear.x - 0.25) < 0.01  # 0.5 * 0.5
            assert abs(twist.angular.z - 0.3) < 0.01   # 0.3 * 1.0
            
        finally:
            # Cleanup
            process.terminate()
            process.wait(timeout=5)
            test_node.destroy_node()
    
    def test_deadzone_filtering(self):
        """Test that values below deadzone are filtered out."""
        test_node = Node("test_deadzone")
        
        joy_pub = test_node.create_publisher(Joy, "/joy", 10)
        received_twists = []
        
        def twist_callback(msg):
            received_twists.append(msg)
        
        twist_sub = test_node.create_subscription(
            Twist, "/cmd_vel", twist_callback, 10
        )
        
        # Start the node
        env = os.environ.copy()
        process = subprocess.Popen(
            ["ros2", "run", "joy_msg_router_rs", "joy_msg_router"],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        try:
            time.sleep(2)
            
            # Send values below deadzone (default is 0.1)
            joy_msg = Joy()
            joy_msg.axes = [0.0, 0.05, 0.0, 0.05]  # Below deadzone
            joy_msg.buttons = []
            
            for _ in range(5):
                joy_pub.publish(joy_msg)
                rclpy.spin_once(test_node, timeout_sec=0.1)
                time.sleep(0.1)
            
            # Check received messages
            if len(received_twists) > 0:
                twist = received_twists[-1]
                assert twist.linear.x == 0.0, "Linear X should be zero (deadzone)"
                assert twist.angular.z == 0.0, "Angular Z should be zero (deadzone)"
            
        finally:
            process.terminate()
            process.wait(timeout=5)
            test_node.destroy_node()
    
    def test_enable_button_with_config(self):
        """Test enable button functionality with configuration file."""
        test_node = Node("test_enable_button")
        
        joy_pub = test_node.create_publisher(Joy, "/joy", 10)
        received_twists = []
        
        def twist_callback(msg):
            received_twists.append(msg)
        
        twist_sub = test_node.create_subscription(
            Twist, "/cmd_vel", twist_callback, 10
        )
        
        # Create a test config file with enable button
        config_content = """
default_profile: "test_enable"
profiles:
  test_enable:
    enable_button: 4  # L1 button
    axis_mappings:
      - joy_axis: 1
        output_field: linear_x
        scale: 1.0
        offset: 0.0
        deadzone: 0.1
"""
        config_path = "/tmp/test_joy_config.yaml"
        with open(config_path, "w") as f:
            f.write(config_content)
        
        # Start the node with config
        env = os.environ.copy()
        process = subprocess.Popen(
            ["ros2", "run", "joy_msg_router_rs", "joy_msg_router", 
             "--ros-args", "-p", f"config_file:={config_path}"],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        try:
            time.sleep(2)
            
            # First test: Send joy without enable button pressed
            joy_msg = Joy()
            joy_msg.axes = [0.0, 0.5, 0.0, 0.0]
            joy_msg.buttons = [0, 0, 0, 0, 0]  # Button 4 not pressed
            
            received_twists.clear()
            for _ in range(3):
                joy_pub.publish(joy_msg)
                rclpy.spin_once(test_node, timeout_sec=0.1)
                time.sleep(0.1)
            
            # Should not receive any twist messages (or zeros)
            if len(received_twists) > 0:
                assert received_twists[-1].linear.x == 0.0
            
            # Second test: Send joy with enable button pressed
            joy_msg.buttons = [0, 0, 0, 0, 1]  # Button 4 pressed
            
            received_twists.clear()
            for _ in range(3):
                joy_pub.publish(joy_msg)
                rclpy.spin_once(test_node, timeout_sec=0.1)
                time.sleep(0.1)
            
            # Should receive twist messages
            assert len(received_twists) > 0, "No Twist messages received with enable button"
            assert abs(received_twists[-1].linear.x - 0.5) < 0.01
            
        finally:
            process.terminate()
            process.wait(timeout=5)
            test_node.destroy_node()
            if os.path.exists(config_path):
                os.remove(config_path)
    
    def test_button_action_publish_twist(self):
        """Test button action that publishes a fixed Twist."""
        test_node = Node("test_button_action")
        
        joy_pub = test_node.create_publisher(Joy, "/joy", 10)
        received_twists = []
        
        def twist_callback(msg):
            received_twists.append(msg)
        
        twist_sub = test_node.create_subscription(
            Twist, "/cmd_vel", twist_callback, 10
        )
        
        # Create config with button action
        config_content = """
default_profile: "test_button"
profiles:
  test_button:
    axis_mappings: []
    button_mappings:
      - button: 0
        action:
          type: publish_twist
          linear_x: 1.0
          linear_y: 0.0
          linear_z: 0.0
          angular_x: 0.0
          angular_y: 0.0
          angular_z: 0.5
"""
        config_path = "/tmp/test_button_config.yaml"
        with open(config_path, "w") as f:
            f.write(config_content)
        
        # Start the node
        env = os.environ.copy()
        process = subprocess.Popen(
            ["ros2", "run", "joy_msg_router_rs", "joy_msg_router",
             "--ros-args", "-p", f"config_file:={config_path}"],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        try:
            time.sleep(2)
            
            # Send joy with button pressed
            joy_msg = Joy()
            joy_msg.axes = []
            joy_msg.buttons = [1, 0, 0, 0]  # Button 0 pressed
            
            for _ in range(3):
                joy_pub.publish(joy_msg)
                rclpy.spin_once(test_node, timeout_sec=0.1)
                time.sleep(0.1)
            
            # Check that we received the configured twist
            assert len(received_twists) > 0
            twist = received_twists[-1]
            assert abs(twist.linear.x - 1.0) < 0.01
            assert abs(twist.angular.z - 0.5) < 0.01
            
        finally:
            process.terminate()
            process.wait(timeout=5)
            test_node.destroy_node()
            if os.path.exists(config_path):
                os.remove(config_path)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])