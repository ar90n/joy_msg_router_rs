#[cfg(test)]
mod tests {
    use crate::config::{ActionType, ButtonMapping, Profile};
    use crate::command_queue::{Command, CommandQueue, Priority, PrioritizedCommand};
    use std::sync::Arc;
    
    #[test]
    fn test_publish_bool_action() {
        let mut profile = Profile::new("test".to_string());
        
        // Add a button mapping that publishes a Bool
        profile.button_mappings.push(ButtonMapping {
            button: 0,
            action: ActionType::PublishBool {
                topic: "/enable_motor".to_string(),
                value: true,
                once: true,
            },
        });
        
        // Test that the configuration is correct
        assert_eq!(profile.button_mappings.len(), 1);
        match &profile.button_mappings[0].action {
            ActionType::PublishBool { topic, value, once } => {
                assert_eq!(topic, "/enable_motor");
                assert_eq!(*value, true);
                assert_eq!(*once, true);
            }
            _ => panic!("Expected PublishBool action"),
        }
    }
    
    #[test]
    fn test_publish_int32_action() {
        let mut profile = Profile::new("test".to_string());
        
        // Add a button mapping that publishes an Int32
        profile.button_mappings.push(ButtonMapping {
            button: 1,
            action: ActionType::PublishInt32 {
                topic: "/set_mode".to_string(),
                value: 42,
                once: false,
            },
        });
        
        // Test that the configuration is correct
        assert_eq!(profile.button_mappings.len(), 1);
        match &profile.button_mappings[0].action {
            ActionType::PublishInt32 { topic, value, once } => {
                assert_eq!(topic, "/set_mode");
                assert_eq!(*value, 42);
                assert_eq!(*once, false);
            }
            _ => panic!("Expected PublishInt32 action"),
        }
    }
    
    #[test]
    fn test_publish_float64_action() {
        let mut profile = Profile::new("test".to_string());
        
        // Add a button mapping that publishes a Float64
        profile.button_mappings.push(ButtonMapping {
            button: 2,
            action: ActionType::PublishFloat64 {
                topic: "/set_speed".to_string(),
                value: 3.14159,
                once: true,
            },
        });
        
        // Test that the configuration is correct
        assert_eq!(profile.button_mappings.len(), 1);
        match &profile.button_mappings[0].action {
            ActionType::PublishFloat64 { topic, value, once } => {
                assert_eq!(topic, "/set_speed");
                assert_eq!(*value, 3.14159);
                assert_eq!(*once, true);
            }
            _ => panic!("Expected PublishFloat64 action"),
        }
    }
    
    #[test]
    fn test_publish_string_action() {
        let mut profile = Profile::new("test".to_string());
        
        // Add a button mapping that publishes a String
        profile.button_mappings.push(ButtonMapping {
            button: 3,
            action: ActionType::PublishString {
                topic: "/set_status".to_string(),
                value: "active".to_string(),
                once: true,
            },
        });
        
        // Test that the configuration is correct
        assert_eq!(profile.button_mappings.len(), 1);
        match &profile.button_mappings[0].action {
            ActionType::PublishString { topic, value, once } => {
                assert_eq!(topic, "/set_status");
                assert_eq!(value, "active");
                assert_eq!(*once, true);
            }
            _ => panic!("Expected PublishString action"),
        }
    }
    
    #[test]
    fn test_publish_twist_stamped_action() {
        let mut profile = Profile::new("test".to_string());
        
        // Add a button mapping that publishes a TwistStamped
        profile.button_mappings.push(ButtonMapping {
            button: 4,
            action: ActionType::PublishTwistStamped {
                linear_x: 1.0,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_x: 0.0,
                angular_y: 0.0,
                angular_z: 0.5,
                frame_id: "base_link".to_string(),
                once: false,
            },
        });
        
        // Test that the configuration is correct
        assert_eq!(profile.button_mappings.len(), 1);
        match &profile.button_mappings[0].action {
            ActionType::PublishTwistStamped { linear_x, angular_z, frame_id, once, .. } => {
                assert_eq!(*linear_x, 1.0);
                assert_eq!(*angular_z, 0.5);
                assert_eq!(frame_id, "base_link");
                assert_eq!(*once, false);
            }
            _ => panic!("Expected PublishTwistStamped action"),
        }
    }
    
    #[test]
    fn test_command_queue_with_new_types() {
        let queue = Arc::new(CommandQueue::new());
        let sender = queue.get_sender();
        
        // Test Bool command
        if let Some(mut bool_msg) = std_msgs::msg::Bool::new() {
            bool_msg.data = true;
            let cmd = PrioritizedCommand {
                command: Command::PublishBool {
                    topic: "/test_bool".to_string(),
                    value: bool_msg,
                },
                priority: Priority::Normal,
            };
            assert!(sender.send(cmd).is_ok());
        }
        
        // Test Int32 command
        if let Some(mut int_msg) = std_msgs::msg::Int32::new() {
            int_msg.data = 123;
            let cmd = PrioritizedCommand {
                command: Command::PublishInt32 {
                    topic: "/test_int".to_string(),
                    value: int_msg,
                },
                priority: Priority::High,
            };
            assert!(sender.send(cmd).is_ok());
        }
        
        // Test Float64 command
        if let Some(mut float_msg) = std_msgs::msg::Float64::new() {
            float_msg.data = 3.14;
            let cmd = PrioritizedCommand {
                command: Command::PublishFloat64 {
                    topic: "/test_float".to_string(),
                    value: float_msg,
                },
                priority: Priority::Normal,
            };
            assert!(sender.send(cmd).is_ok());
        }
        
        // Process commands using the process_pending method to ensure priority ordering
        let mut commands = Vec::new();
        queue.process_pending(|cmd| {
            commands.push(cmd);
            Ok(())
        }).unwrap();
        
        // Should have 3 commands, with Int32 first (high priority)
        assert_eq!(commands.len(), 3);
        match &commands[0].command {
            Command::PublishInt32 { topic, .. } => assert_eq!(topic, "/test_int"),
            _ => panic!("Expected PublishInt32 command first (high priority)"),
        }
    }
}