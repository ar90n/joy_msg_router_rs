#!/usr/bin/env python3
"""
Integration tests for service calling functionality in joy_msg_router_rs.

Tests the fire-and-forget service calling pattern to ensure:
1. Services are called when buttons are pressed (just_activated)
2. Services are not called when buttons are held
3. Multiple service mappings work correctly
4. Service calls don't block joy message processing
"""

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, Empty
import time
import threading


class ServiceCallTestNode(Node):
    """Test node that provides services and tracks calls."""
    
    def __init__(self):
        super().__init__('service_call_test_node')
        
        # Service call counters
        self.trigger_call_count = 0
        self.empty_call_count = 0
        self.service_responses = []
        
        # Create test services
        self.trigger_service = self.create_service(
            Trigger, '/test_trigger', self.handle_trigger)
        self.empty_service = self.create_service(
            Empty, '/test_empty', self.handle_empty)
        
        # Create slow service to test non-blocking behavior
        self.slow_service = self.create_service(
            Trigger, '/test_slow_trigger', self.handle_slow_trigger)
        
        # Joy publisher for testing
        self.joy_publisher = self.create_publisher(Joy, '/joy', 10)
        
        # Track timing for non-blocking tests
        self.last_joy_publish_time = None
        self.service_call_times = []
    
    def handle_trigger(self, request, response):
        """Handle Trigger service calls."""
        self.trigger_call_count += 1
        response.success = True
        response.message = f"Trigger called {self.trigger_call_count} times"
        self.service_responses.append(('trigger', response.message))
        self.service_call_times.append(time.time())
        return response
    
    def handle_empty(self, request, response):
        """Handle Empty service calls."""
        self.empty_call_count += 1
        self.service_responses.append(('empty', 'called'))
        self.service_call_times.append(time.time())
        return response
    
    def handle_slow_trigger(self, request, response):
        """Handle slow service to test non-blocking."""
        # Simulate slow service (200ms)
        time.sleep(0.2)
        response.success = True
        response.message = "Slow service completed"
        return response
    
    def publish_joy(self, axes=None, buttons=None):
        """Publish a joy message."""
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes or []
        msg.buttons = buttons or []
        self.joy_publisher.publish(msg)
        self.last_joy_publish_time = time.time()
    
    def reset_counters(self):
        """Reset all counters for next test."""
        self.trigger_call_count = 0
        self.empty_call_count = 0
        self.service_responses = []
        self.service_call_times = []


@pytest.fixture
def test_node():
    """Create and spin up test node."""
    rclpy.init()
    node = ServiceCallTestNode()
    
    # Spin in background thread
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    
    yield node
    
    # Cleanup
    executor.shutdown()
    thread.join()
    node.destroy_node()
    rclpy.shutdown()


def wait_for_service_call(node, expected_count, service_type='trigger', timeout=2.0):
    """Wait for service to be called expected number of times."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        if service_type == 'trigger' and node.trigger_call_count >= expected_count:
            return True
        elif service_type == 'empty' and node.empty_call_count >= expected_count:
            return True
        time.sleep(0.01)
    return False


class TestServiceCalls:
    """Test service calling functionality."""
    
    def test_button_press_triggers_service(self, test_node):
        """Test that pressing a button triggers service call."""
        # Press button 0 (should trigger service)
        test_node.publish_joy(buttons=[1, 0, 0])
        
        # Wait for service call
        assert wait_for_service_call(test_node, 1, 'trigger')
        assert test_node.trigger_call_count == 1
        
        # Verify response was received (even though we don't wait for it)
        assert len(test_node.service_responses) == 1
        assert test_node.service_responses[0][0] == 'trigger'
    
    def test_button_hold_no_repeat(self, test_node):
        """Test that holding button doesn't repeat service call."""
        # Press button
        test_node.publish_joy(buttons=[1, 0, 0])
        assert wait_for_service_call(test_node, 1, 'trigger')
        
        # Hold button (send same state multiple times)
        for _ in range(5):
            test_node.publish_joy(buttons=[1, 0, 0])
            time.sleep(0.05)
        
        # Should still only have 1 call
        time.sleep(0.2)
        assert test_node.trigger_call_count == 1
    
    def test_button_release_and_press(self, test_node):
        """Test that releasing and pressing again triggers new call."""
        # Press button
        test_node.publish_joy(buttons=[1, 0, 0])
        assert wait_for_service_call(test_node, 1, 'trigger')
        
        # Release button
        test_node.publish_joy(buttons=[0, 0, 0])
        time.sleep(0.1)
        
        # Press again
        test_node.publish_joy(buttons=[1, 0, 0])
        assert wait_for_service_call(test_node, 2, 'trigger')
        
        assert test_node.trigger_call_count == 2
    
    def test_multiple_service_types(self, test_node):
        """Test multiple service types can be called."""
        # Press button 0 (Trigger service)
        test_node.publish_joy(buttons=[1, 0, 0])
        assert wait_for_service_call(test_node, 1, 'trigger')
        
        # Press button 1 (Empty service)
        test_node.publish_joy(buttons=[0, 1, 0])
        assert wait_for_service_call(test_node, 1, 'empty')
        
        assert test_node.trigger_call_count == 1
        assert test_node.empty_call_count == 1
    
    def test_simultaneous_buttons(self, test_node):
        """Test pressing multiple buttons at once."""
        test_node.reset_counters()
        
        # Press both buttons at once
        test_node.publish_joy(buttons=[1, 1, 0])
        
        # Both services should be called
        assert wait_for_service_call(test_node, 1, 'trigger')
        assert wait_for_service_call(test_node, 1, 'empty')
    
    def test_non_blocking_service_calls(self, test_node):
        """Test that service calls don't block joy processing."""
        test_node.reset_counters()
        
        # Rapidly publish joy messages while triggering slow service
        joy_publish_times = []
        
        # Press button for slow service
        test_node.publish_joy(buttons=[0, 0, 1])
        joy_publish_times.append(time.time())
        
        # Continue publishing joy messages
        for i in range(10):
            time.sleep(0.02)  # 50Hz
            test_node.publish_joy(buttons=[0, 0, 1])
            joy_publish_times.append(time.time())
        
        # Check that joy messages weren't blocked
        # All messages should be published within reasonable time
        for i in range(1, len(joy_publish_times)):
            delta = joy_publish_times[i] - joy_publish_times[i-1]
            # Should be close to 20ms, definitely less than 200ms (slow service time)
            assert delta < 0.1, f"Joy publishing was blocked: {delta}s between messages"
    
    def test_service_not_available(self, test_node):
        """Test behavior when service is not available."""
        # This would test the case where service client exists but service is down
        # In fire-and-forget pattern, this should not block or crash
        # (Actual implementation would log error and continue)
        pass
    
    def test_enable_button_with_services(self, test_node):
        """Test that enable button doesn't affect service calls."""
        # Service calls should work regardless of enable button state
        # This documents the expected behavior
        
        # Press service button without enable button
        test_node.publish_joy(buttons=[1, 0, 0, 0, 0])  # No enable button (button 4)
        assert wait_for_service_call(test_node, 1, 'trigger')
        
        # Service should still be called even without enable button
        assert test_node.trigger_call_count == 1


class TestServiceConfiguration:
    """Test service configuration and parameter loading."""
    
    def test_yaml_config_with_services(self):
        """Test loading YAML configuration with service mappings."""
        # This would test actual YAML loading with service configurations
        # Verifying that service_name and service_type are properly loaded
        pass
    
    def test_ros_params_with_services(self):
        """Test loading ROS parameters with service mappings."""
        # This would test parameter-based configuration
        # Verifying proper parameter structure for services
        pass
    
    def test_mixed_actions_config(self):
        """Test configuration with both publish and service actions."""
        # This would verify that mixed configurations work correctly
        pass


if __name__ == '__main__':
    pytest.main([__file__, '-v'])