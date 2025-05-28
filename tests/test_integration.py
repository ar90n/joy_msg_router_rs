#!/usr/bin/env python3
"""
Integration tests for joy_msg_router using launch_testing.
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import launch
import launch_ros
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description for testing."""
    joy_router_node = launch_ros.actions.Node(
        package='joy_msg_router_rs',
        executable='joy_msg_router',
        name='joy_msg_router',
        output='screen',
    )
    
    return (
        launch.LaunchDescription([
            joy_router_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'joy_router': joy_router_node,
        }
    )


class TestJoyMsgRouter(unittest.TestCase):
    """Test cases for joy_msg_router integration."""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 context."""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 context."""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up test node."""
        self.node = Node('test_joy_router_integration')
        self.joy_pub = self.node.create_publisher(Joy, '/joy', 10)
        self.received_twists = []
        
        def twist_callback(msg):
            self.received_twists.append(msg)
        
        self.twist_sub = self.node.create_subscription(
            Twist, '/cmd_vel', twist_callback, 10
        )
        
        # Allow time for connections
        time.sleep(1.0)
    
    def tearDown(self):
        """Clean up test node."""
        self.node.destroy_node()
    
    def test_basic_joy_to_twist_conversion(self):
        """Test basic conversion from Joy to Twist messages."""
        # Create a joy message
        joy_msg = Joy()
        # Default config uses axis 1 for linear.x (scale 0.5)
        # and axis 3 for angular.z (scale 1.0)
        joy_msg.axes = [0.0, 0.6, 0.0, 0.4, 0.0]
        joy_msg.buttons = []
        
        # Clear received messages
        self.received_twists.clear()
        
        # Publish joy message multiple times
        for _ in range(10):
            self.joy_pub.publish(joy_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)
        
        # Verify we received twist messages
        self.assertGreater(len(self.received_twists), 0, 
                          "No Twist messages received")
        
        # Check the last received twist
        last_twist = self.received_twists[-1]
        expected_linear_x = 0.6 * 0.5  # axis value * scale
        expected_angular_z = 0.4 * 1.0  # axis value * scale
        
        self.assertAlmostEqual(last_twist.linear.x, expected_linear_x, places=2)
        self.assertAlmostEqual(last_twist.angular.z, expected_angular_z, places=2)
    
    def test_deadzone_filtering(self):
        """Test that values within deadzone are filtered."""
        # Create joy message with values below deadzone (0.1)
        joy_msg = Joy()
        joy_msg.axes = [0.0, 0.05, 0.0, 0.08, 0.0]
        joy_msg.buttons = []
        
        self.received_twists.clear()
        
        # Publish joy message
        for _ in range(10):
            self.joy_pub.publish(joy_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)
        
        # If we receive any twists, they should have zero velocity
        if len(self.received_twists) > 0:
            last_twist = self.received_twists[-1]
            self.assertEqual(last_twist.linear.x, 0.0)
            self.assertEqual(last_twist.angular.z, 0.0)
    
    def test_zero_joy_input(self):
        """Test handling of zero joy inputs."""
        # Create joy message with all zeros
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.buttons = [0] * 10
        
        self.received_twists.clear()
        
        # Publish joy message
        for _ in range(5):
            self.joy_pub.publish(joy_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)
        
        # Should receive twist messages with zero velocities
        if len(self.received_twists) > 0:
            last_twist = self.received_twists[-1]
            self.assertEqual(last_twist.linear.x, 0.0)
            self.assertEqual(last_twist.linear.y, 0.0)
            self.assertEqual(last_twist.linear.z, 0.0)
            self.assertEqual(last_twist.angular.x, 0.0)
            self.assertEqual(last_twist.angular.y, 0.0)
            self.assertEqual(last_twist.angular.z, 0.0)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Test process output after shutdown."""
    
    def test_exit_code(self, proc_info, joy_router):
        """Check that the process exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info, [0, -2, -15], joy_router)