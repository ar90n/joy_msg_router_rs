use crate::command_queue::{Command, PrioritizedCommand, Priority};
use crate::timer::TimerCallback;
use crate::config::ActionType;
use geometry_msgs::msg::Twist;
use std::sync::mpsc::Sender;
use safe_drive::logger::Logger;
use safe_drive::pr_debug;

/// Timer callback for sending periodic Twist commands
pub struct TwistTimerCallback {
    command_sender: Sender<PrioritizedCommand>,
    twist: Twist,
    priority: Priority,
    logger: Logger,
}

impl TwistTimerCallback {
    pub fn new(command_sender: Sender<PrioritizedCommand>, twist: Twist, priority: Priority, logger: Logger) -> Self {
        Self {
            command_sender,
            twist,
            priority,
            logger,
        }
    }
}

impl TimerCallback for TwistTimerCallback {
    fn on_timer(&mut self) {
        pr_debug!(self.logger, "Timer callback triggered for Twist command");
        
        // Create a new Twist with the same values
        if let Some(mut twist_copy) = Twist::new() {
            twist_copy.linear.x = self.twist.linear.x;
            twist_copy.linear.y = self.twist.linear.y;
            twist_copy.linear.z = self.twist.linear.z;
            twist_copy.angular.x = self.twist.angular.x;
            twist_copy.angular.y = self.twist.angular.y;
            twist_copy.angular.z = self.twist.angular.z;
            
            let command = PrioritizedCommand {
                command: Command::PublishTwist(twist_copy),
                priority: self.priority,
            };
            
            if let Err(e) = self.command_sender.send(command) {
                pr_debug!(self.logger, "Failed to send timer command: {:?}", e);
            }
        }
    }
}

/// Timer callback for sending stop commands
pub struct StopTimerCallback {
    command_sender: Sender<PrioritizedCommand>,
    logger: Logger,
}

impl StopTimerCallback {
    pub fn new(command_sender: Sender<PrioritizedCommand>, logger: Logger) -> Self {
        Self {
            command_sender,
            logger,
        }
    }
}

impl TimerCallback for StopTimerCallback {
    fn on_timer(&mut self) {
        pr_debug!(self.logger, "Timer callback triggered for Stop command");
        
        let command = PrioritizedCommand {
            command: Command::Stop,
            priority: Priority::High,
        };
        
        if let Err(e) = self.command_sender.send(command) {
            pr_debug!(self.logger, "Failed to send stop command: {:?}", e);
        }
    }
}

/// Timer callback for service calls
pub struct ServiceTimerCallback {
    command_sender: Sender<PrioritizedCommand>,
    service_name: String,
    service_type: String,
    priority: Priority,
    logger: Logger,
}

impl ServiceTimerCallback {
    pub fn new(
        command_sender: Sender<PrioritizedCommand>,
        service_name: String,
        service_type: String,
        priority: Priority,
        logger: Logger,
    ) -> Self {
        Self {
            command_sender,
            service_name,
            service_type,
            priority,
            logger,
        }
    }
}

impl TimerCallback for ServiceTimerCallback {
    fn on_timer(&mut self) {
        pr_debug!(self.logger, "Timer callback triggered for service: {}", self.service_name);
        
        let command = PrioritizedCommand {
            command: Command::CallService {
                service_name: self.service_name.clone(),
                service_type: self.service_type.clone(),
            },
            priority: self.priority,
        };
        
        if let Err(e) = self.command_sender.send(command) {
            pr_debug!(self.logger, "Failed to send service command: {:?}", e);
        }
    }
}

/// Create appropriate timer callback based on action type
pub fn create_timer_callback(
    action: &ActionType,
    command_sender: Sender<PrioritizedCommand>,
    logger: Logger,
) -> Option<Box<dyn TimerCallback>> {
    match action {
        ActionType::PublishTwist { linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, .. } => {
            if let Some(mut twist) = Twist::new() {
                twist.linear.x = *linear_x;
                twist.linear.y = *linear_y;
                twist.linear.z = *linear_z;
                twist.angular.x = *angular_x;
                twist.angular.y = *angular_y;
                twist.angular.z = *angular_z;
                
                Some(Box::new(TwistTimerCallback::new(
                    command_sender,
                    twist,
                    Priority::Normal,
                    logger,
                )))
            } else {
                None
            }
        }
        ActionType::CallService { service_name, service_type, .. } => {
            Some(Box::new(ServiceTimerCallback::new(
                command_sender,
                service_name.clone(),
                service_type.clone(),
                Priority::Normal,
                logger,
            )))
        }
        ActionType::NoAction => {
            Some(Box::new(StopTimerCallback::new(command_sender, logger)))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command_queue::CommandQueue;
    use std::sync::Arc;
    
    #[test]
    fn test_twist_timer_callback() {
        let queue = Arc::new(CommandQueue::new());
        let sender = queue.get_sender();
        let logger = Logger::new("test");
        
        let mut twist = Twist::new().unwrap();
        twist.linear.x = 1.0;
        
        let mut callback = TwistTimerCallback::new(sender, twist, Priority::Normal, logger);
        
        // Trigger the callback
        callback.on_timer();
        
        // Check that command was queued
        let received = queue.try_dequeue();
        assert!(received.is_some());
        
        let cmd = received.unwrap();
        match cmd.command {
            Command::PublishTwist(t) => assert_eq!(t.linear.x, 1.0),
            _ => panic!("Expected PublishTwist command"),
        }
    }
    
    #[test]
    fn test_stop_timer_callback() {
        let queue = Arc::new(CommandQueue::new());
        let sender = queue.get_sender();
        let logger = Logger::new("test");
        
        let mut callback = StopTimerCallback::new(sender, logger);
        
        // Trigger the callback
        callback.on_timer();
        
        // Check that stop command was queued
        let received = queue.try_dequeue();
        assert!(received.is_some());
        
        let cmd = received.unwrap();
        assert!(matches!(cmd.command, Command::Stop));
        assert_eq!(cmd.priority, Priority::High);
    }
    
    #[test]
    fn test_create_timer_callback() {
        let queue = Arc::new(CommandQueue::new());
        let sender = queue.get_sender();
        let logger = Logger::new("test");
        
        // Test Twist action
        let action = ActionType::PublishTwist {
            linear_x: 2.0,
            linear_y: 0.0,
            linear_z: 0.0,
            angular_x: 0.0,
            angular_y: 0.0,
            angular_z: 0.0,
            once: false,
        };
        
        let callback = create_timer_callback(&action, sender.clone(), Logger::new("test"));
        assert!(callback.is_some());
        
        // Test Service action
        let action = ActionType::CallService {
            service_name: "test_service".to_string(),
            service_type: "std_srvs/srv/Empty".to_string(),
            once: false,
        };
        
        let callback = create_timer_callback(&action, sender.clone(), Logger::new("test"));
        assert!(callback.is_some());
        
        // Test NoAction
        let action = ActionType::NoAction;
        let callback = create_timer_callback(&action, sender, logger);
        assert!(callback.is_some());
    }
}