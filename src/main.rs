use safe_drive::{
    context::Context as SafeDriveContext, logger::Logger, pr_debug, pr_error, pr_info, pr_warn,
};
use std::sync::{Arc, Mutex};

use geometry_msgs::msg::Twist;
use sensor_msgs::msg::Joy;

mod clients;
mod config;
mod joy_msg_tracker;
mod logging;
mod profile;
mod publishers;

use anyhow::Context;
use anyhow::{anyhow, Result};
use clients::ServiceClients;
use config::{ActionType, InputMapping, InputSource};
use joy_msg_tracker::JoyMsgTracker;
use logging::{log_error, LogContext};
use profile::{is_enabled, load_profile_from_params};

fn main() -> Result<()> {
    main_impl()
}

fn process_input_mappings(
    tracker: &JoyMsgTracker,
    input_mappings: &[InputMapping],
    publishers: &publishers::Publishers,
    clients: &ServiceClients,
    logger: &Logger,
) -> Result<()> {
    // Collect Twist contributions per topic
    let mut twist_accumulators: std::collections::HashMap<String, Twist> = std::collections::HashMap::new();

    for mapping in input_mappings {
        let (input_value, is_active, just_activated) = match mapping.source {
            InputSource::Axis(idx) => {
                let value = tracker.get_axis(idx).unwrap_or(0.0) as f64;
                let processed = mapping.process_value(value);
                (value, processed.abs() > 0.0, false)
            }
            InputSource::Button(idx) => {
                let pressed = tracker.is_pressed(idx);
                let just_pressed = tracker.just_pressed(idx);
                let value = if pressed { 1.0 } else { 0.0 };
                (value, pressed, just_pressed)
            }
        };

        if !is_active {
            continue; // Skip inactive mappings
        }

        // Calculate processed value with scale, offset, and deadzone
        let processed_value = mapping.process_value(input_value);

        match &mapping.action {
            ActionType::Publish {
                topic,
                message_type,
                field,
                once,
            } if is_active && (!*once || just_activated) => {
                // Special handling for Twist messages - accumulate values
                if message_type == "geometry_msgs/msg/Twist" {
                    let twist = twist_accumulators.entry(topic.clone())
                        .or_insert_with(|| Twist::new().unwrap());
                    
                    if let Some(field_name) = field {
                        match field_name.as_str() {
                            "linear.x" | "linear_x" => twist.linear.x += processed_value,
                            "linear.y" | "linear_y" => twist.linear.y += processed_value,
                            "linear.z" | "linear_z" => twist.linear.z += processed_value,
                            "angular.x" | "angular_x" => twist.angular.x += processed_value,
                            "angular.y" | "angular_y" => twist.angular.y += processed_value,
                            "angular.z" | "angular_z" => twist.angular.z += processed_value,
                            _ => {
                                pr_error!(logger, "Unknown twist field: {}", field_name);
                            }
                        }
                    }
                } else {
                    // For non-Twist messages, publish immediately
                    if let Err(e) = publishers.publish_value(topic, processed_value, field.as_deref()) {
                        log_error(
                            logger,
                            LogContext {
                                module: "main",
                                function: "process_input_mappings",
                                details: Some("generic_publish"),
                            },
                            &e,
                        );
                    }
                }
            }
            ActionType::CallService {
                service_name,
                service_type,
            } if just_activated => {
                // For now, we just log the service call
                // Full service support requires proper ROS service message types
                if clients.has_service(service_name) {
                    pr_info!(
                        logger,
                        "Would call service: {} (type: {})",
                        service_name,
                        service_type
                    );
                    
                    // TODO: Implement actual service calls when std_srvs types are available
                } else {
                    pr_warn!(
                        logger,
                        "Service {} not configured in clients",
                        service_name
                    );
                }
            }
            _ => {}
        }
    }

    // Publish all accumulated Twist messages
    for (topic, twist) in twist_accumulators {
        if let Err(e) = publishers.publish_twist(&topic, &twist) {
            log_error(
                logger,
                LogContext {
                    module: "main",
                    function: "process_input_mappings",
                    details: Some("accumulated_twist"),
                },
                &e,
            );
        }
    }

    Ok(())
}

fn main_impl() -> Result<()> {
    let logger = Logger::new("joy_msg_router");
    let ctx =
        SafeDriveContext::new().map_err(|e| anyhow!("Failed to create ROS2 context: {:?}", e))?;
    let node = Arc::new(
        ctx.create_node("joy_msg_router", None, Default::default())
            .map_err(|e| anyhow!("Failed to create ROS2 node: {:?}", e))?,
    );
    let profile = node
        .create_parameter_server()
        .map_err(|e| anyhow!("Failed to create parameter server: {:?}", e))
        .and_then(|ps| load_profile_from_params(&ps))?;

    pr_info!(logger, "Joy message router node started");
    pr_info!(logger, "Active profile: {}", profile.name);

    let joy_subscriber = node
        .create_subscriber::<Joy>("joy", None)
        .map_err(|e| anyhow!("Failed to create joy subscriber: {:?}", e))?;

    let publishers = 
        Arc::new(publishers::Publishers::from_profile(&node, &profile).context("Failed to create publishers")?);
    
    let clients = 
        Arc::new(ServiceClients::from_profile(&node, &profile).context("Failed to create service clients")?);

    let joy_tracker = Arc::new(Mutex::new(JoyMsgTracker::new()));
    let joy_tracker_sub = Arc::clone(&joy_tracker);

    let mut selector = ctx
        .create_selector()
        .map_err(|e| anyhow!("Failed to create selector: {:?}", e))?;

    selector.add_subscriber(
        joy_subscriber,
        Box::new(move |msg| {
            let sub_logger = Logger::new("joy_msg_router");
            pr_debug!(sub_logger, "Received Joy message");

            let Ok(mut tracker) = joy_tracker_sub.lock() else {
                pr_error!(sub_logger, "Failed to lock joy tracker");
                return;
            };
            tracker.update(&msg);
        }),
    );

    loop {
        selector
            .wait_timeout(std::time::Duration::from_millis(10))
            .map_err(|e| anyhow!("Selector wait failed: {:?}", e))?;

        let Ok(tracker) = joy_tracker.lock() else {
            pr_error!(logger, "Failed to lock joy tracker");
            continue;
        };
        if is_enabled(&profile, &tracker) {
            process_input_mappings(&tracker, &profile.input_mappings, &publishers, &clients, &logger)?;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{ActionType, InputMapping, InputSource, Profile};
    use crate::joy_msg_tracker::JoyMsgTracker;
    use crate::profile::is_enabled;

    fn create_test_joy(axes: Vec<f32>, buttons: Vec<i32>) -> Joy {
        let mut joy = Joy::new().unwrap();
        joy.axes = safe_drive::msg::F32Seq::new(axes.len()).unwrap();
        joy.axes
            .iter_mut()
            .enumerate()
            .for_each(|(i, v)| *v = axes[i]);
        joy.buttons = safe_drive::msg::I32Seq::new(buttons.len()).unwrap();
        joy.buttons
            .iter_mut()
            .enumerate()
            .for_each(|(i, v)| *v = buttons[i]);
        joy
    }

    #[test]
    fn test_is_enabled_no_button() {
        let _joy = create_test_joy(vec![], vec![0, 1, 0]);
        let profile = Profile::new("test".to_string()); // No enable button
        let tracker = Arc::new(Mutex::new(JoyMsgTracker::new()));
        {
            let mut tracker_guard = tracker.lock().unwrap();
            tracker_guard.update_buttons(&[0, 1, 0]);
        }
        let tracker_guard = tracker.lock().unwrap();

        assert!(is_enabled(&profile, &tracker_guard));
    }

    #[test]
    fn test_is_enabled_with_button() {
        let _joy = create_test_joy(vec![], vec![0, 1, 0]);
        let mut profile = Profile::new("test".to_string());
        let tracker = Arc::new(Mutex::new(JoyMsgTracker::new()));
        {
            let mut tracker_guard = tracker.lock().unwrap();
            tracker_guard.update_buttons(&[0, 1, 0]);
        }

        profile.enable_button = Some(1);
        {
            let tracker_guard = tracker.lock().unwrap();
            assert!(is_enabled(&profile, &tracker_guard)); // Button 1 is pressed
        }

        profile.enable_button = Some(0);
        {
            let tracker_guard = tracker.lock().unwrap();
            assert!(!is_enabled(&profile, &tracker_guard)); // Button 0 is not pressed
        }

        profile.enable_button = Some(5);
        {
            let tracker_guard = tracker.lock().unwrap();
            assert!(!is_enabled(&profile, &tracker_guard)); // Button 5 doesn't exist
        }
    }

    #[test]
    fn test_is_enabled_multiple_buttons() {
        let _joy = create_test_joy(vec![], vec![0, 0, 1, 0]);
        let mut profile = Profile::new("test".to_string());
        let tracker = Arc::new(Mutex::new(JoyMsgTracker::new()));
        {
            let mut tracker_guard = tracker.lock().unwrap();
            tracker_guard.update_buttons(&[0, 0, 1, 0]);
        }

        // Test single enable button with different indices
        profile.enable_button = Some(2);
        {
            let tracker_guard = tracker.lock().unwrap();
            assert!(is_enabled(&profile, &tracker_guard)); // Button 2 is pressed
        }

        profile.enable_button = Some(0);
        {
            let tracker_guard = tracker.lock().unwrap();
            assert!(!is_enabled(&profile, &tracker_guard)); // Button 0 not pressed
        }
    }

    #[test]
    fn test_button_to_twist_field() {
        // Test mapping a button to publish a twist field value
        let mut profile = Profile::new("test".to_string());
        
        // Button 0 -> emergency stop (zero linear_x)
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(0),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
            },
            scale: 0.0,  // Zero for stop
            offset: 0.0,
            deadzone: 0.0,
        });
        
        // Button 1 -> fixed forward speed
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(1),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
            },
            scale: 0.5,  // Fixed speed
            offset: 0.0,
            deadzone: 0.0,
        });
        
        // Button 2 -> rotate left
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(2),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("angular.z".to_string()),
                once: false,
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        let tracker = JoyMsgTracker::new();
        
        // Test button 0 pressed - should output 0
        let mut tracker_test = tracker.clone();
        tracker_test.update_buttons(&[1, 0, 0]);
        let mapping = &profile.input_mappings[0];
        let value = mapping.process_input(&tracker_test);
        assert_eq!(value, 0.0); // scale = 0.0
        
        // Test button 1 pressed - should output 0.5
        tracker_test.update_buttons(&[0, 1, 0]);
        let mapping = &profile.input_mappings[1];
        let value = mapping.process_input(&tracker_test);
        assert_eq!(value, 0.5); // scale = 0.5
        
        // Test button 2 pressed - should output 1.0
        tracker_test.update_buttons(&[0, 0, 1]);
        let mapping = &profile.input_mappings[2];
        let value = mapping.process_input(&tracker_test);
        assert_eq!(value, 1.0); // scale = 1.0
    }

    #[test]
    fn test_axis_to_bool_action() {
        // Test using axis as a trigger for boolean action (threshold-based)
        let mut profile = Profile::new("test".to_string());
        
        // Axis 2 (trigger) -> publish bool when pressed beyond threshold
        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(2),
            action: ActionType::Publish {
                topic: "/trigger/pressed".to_string(),
                message_type: "std_msgs/msg/Bool".to_string(),
                field: None,
                once: true,
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.5,  // High deadzone acts as threshold
        });
        
        let mut tracker = JoyMsgTracker::new();
        
        // Test axis below threshold
        tracker.update_axes(&[0.0, 0.0, 0.3]);
        let mapping = &profile.input_mappings[0];
        let value = mapping.process_input(&tracker);
        assert_eq!(value, 0.0); // Below deadzone
        
        // Test axis above threshold
        tracker.update_axes(&[0.0, 0.0, 0.7]);
        let value = mapping.process_input(&tracker);
        assert!(value > 0.0); // Above deadzone, would trigger bool publish
    }

    #[test]
    fn test_multiple_axes_to_twist() {
        // Test multiple axes controlling different twist fields
        let mut profile = Profile::new("test".to_string());
        
        // Axis 0 -> linear.x (forward/back)
        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.1,
        });
        
        // Axis 1 -> linear.y (strafe for holonomic)
        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(1),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.y".to_string()),
                once: false,
            },
            scale: 0.8,
            offset: 0.0,
            deadzone: 0.1,
        });
        
        // Axis 3 -> angular.z (rotation)
        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(3),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("angular.z".to_string()),
                once: false,
            },
            scale: 2.0,
            offset: 0.0,
            deadzone: 0.15,
        });
        
        let mut tracker = JoyMsgTracker::new();
        tracker.update_axes(&[0.5, -0.3, 0.0, 0.8, 0.0]);
        
        // Test each axis mapping
        assert_eq!(profile.input_mappings[0].process_input(&tracker), 0.5 * 1.0);  // Axis 0
        assert!((profile.input_mappings[1].process_input(&tracker) - (-0.3 * 0.8)).abs() < 0.0001); // Axis 1
        assert!((profile.input_mappings[2].process_input(&tracker) - (0.8 * 2.0)).abs() < 0.0001);  // Axis 3
    }

    #[test]
    fn test_axis_with_offset() {
        // Test axis mapping with offset (useful for triggers)
        let mut profile = Profile::new("test".to_string());
        
        // Trigger axis that rests at 1.0 and goes to -1.0 when pressed
        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(5),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
            },
            scale: -0.5,
            offset: 0.5,
            deadzone: 0.1,
        });
        
        let mut tracker = JoyMsgTracker::new();
        
        // Trigger at rest (1.0)
        tracker.update_axes(&[0.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
        let value = profile.input_mappings[0].process_input(&tracker);
        assert_eq!(value, 1.0 * -0.5 + 0.5); // = 0.0
        
        // Trigger fully pressed (-1.0)
        tracker.update_axes(&[0.0, 0.0, 0.0, 0.0, 0.0, -1.0]);
        let value = profile.input_mappings[0].process_input(&tracker);
        assert_eq!(value, -1.0 * -0.5 + 0.5); // = 1.0
    }

    #[test]
    fn test_button_combinations() {
        // Test multiple buttons mapped to same field (additive)
        let mut profile = Profile::new("test".to_string());
        
        // D-pad style control
        // Button 0 -> forward
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(0),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
            },
            scale: 0.3,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        // Button 1 -> backward
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(1),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
            },
            scale: -0.3,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        // Button 2 -> turn left
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(2),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("angular.z".to_string()),
                once: false,
            },
            scale: 0.5,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        // Button 3 -> turn right
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(3),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("angular.z".to_string()),
                once: false,
            },
            scale: -0.5,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        let mut tracker = JoyMsgTracker::new();
        
        // Test forward + turn left
        tracker.update_buttons(&[1, 0, 1, 0]);
        assert_eq!(profile.input_mappings[0].process_input(&tracker), 0.3);  // Forward
        assert_eq!(profile.input_mappings[2].process_input(&tracker), 0.5);  // Turn left
        
        // Test backward + turn right
        tracker.update_buttons(&[0, 1, 0, 1]);
        assert_eq!(profile.input_mappings[1].process_input(&tracker), -0.3); // Backward
        assert_eq!(profile.input_mappings[3].process_input(&tracker), -0.5); // Turn right
    }

    #[test]
    fn test_service_call_action() {
        // Test button triggering service call
        let mut profile = Profile::new("test".to_string());
        
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(5),
            action: ActionType::CallService {
                service_name: "/emergency_stop".to_string(),
                service_type: "std_srvs/srv/Trigger".to_string(),
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        let mut tracker = JoyMsgTracker::new();
        
        // Button not pressed
        tracker.update_buttons(&[0, 0, 0, 0, 0, 0]);
        assert!(!tracker.just_pressed(5));
        
        // Button just pressed
        tracker.update_buttons(&[0, 0, 0, 0, 0, 1]);
        assert!(tracker.just_pressed(5));
        
        // Button held (should not trigger again)
        tracker.update_buttons(&[0, 0, 0, 0, 0, 1]);
        assert!(!tracker.just_pressed(5));
    }
    
    #[test]
    fn test_service_clients_creation() {
        use crate::clients::ServiceClients;
        
        let mappings = vec![
            // Add a supported service (Trigger)
            InputMapping {
                source: InputSource::Button(0),
                action: ActionType::CallService {
                    service_name: "/reset_odometry".to_string(),
                    service_type: "std_srvs/srv/Trigger".to_string(),
                },
                scale: 1.0,
                offset: 0.0,
                deadzone: 0.0,
            },
            // Add an unsupported service type (should be ignored)
            InputMapping {
                source: InputSource::Button(1),
                action: ActionType::CallService {
                    service_name: "/set_mode".to_string(),
                    service_type: "custom_srvs/srv/SetMode".to_string(),
                },
                scale: 1.0,
                offset: 0.0,
                deadzone: 0.0,
            },
        ];
        
        let clients = ServiceClients::from_mappings(&mappings);
        
        // Check that only the Trigger service was registered
        assert!(clients.has_service("/reset_odometry"));
        assert!(!clients.has_service("/set_mode"));
        assert_eq!(clients.trigger_services.len(), 1);
    }

    #[test]
    fn test_deadzone_behavior() {
        let mapping = InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.2,
        };
        
        let mut tracker = JoyMsgTracker::new();
        
        // Values within deadzone should return 0
        for value in &[0.0, 0.1, -0.1, 0.19, -0.19] {
            tracker.update_axes(&[*value]);
            assert_eq!(mapping.process_input(&tracker), 0.0);
        }
        
        // Values outside deadzone should be scaled
        tracker.update_axes(&[0.5]);
        assert_eq!(mapping.process_input(&tracker), 0.5);
        
        tracker.update_axes(&[-0.5]);
        assert_eq!(mapping.process_input(&tracker), -0.5);
    }
    
    #[test]
    fn test_generic_publish_float64() {
        let mapping = InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::Publish {
                topic: "/test/speed".to_string(),
                message_type: "std_msgs/msg/Float64".to_string(),
                field: None,
                once: false,
            },
            scale: 0.5,
            offset: 0.0,
            deadzone: 0.0,
        };
        
        let mut tracker = JoyMsgTracker::new();
        tracker.update_axes(&[1.0]);
        
        // Should scale the input value
        assert_eq!(mapping.process_value(1.0), 0.5);
    }
    
    #[test]
    fn test_generic_publish_vector3() {
        let mapping = InputMapping {
            source: InputSource::Button(0),
            action: ActionType::Publish {
                topic: "/test/vector".to_string(),
                message_type: "geometry_msgs/msg/Vector3".to_string(),
                field: Some("x".to_string()),
                once: true,
            },
            scale: 5.0,
            offset: 0.0,
            deadzone: 0.0,
        };
        
        let mut tracker = JoyMsgTracker::new();
        tracker.update_buttons(&[1]);
        
        // Button pressed should produce scaled value
        assert_eq!(mapping.process_value(1.0), 5.0);
    }
}
