use safe_drive::{context::Context, error::DynError, logger::Logger, pr_debug, pr_info};
use std::sync::{Arc, Mutex};
use std::collections::HashMap;

// Import ROS message types
use geometry_msgs::msg::Twist;
use sensor_msgs::msg::Joy;

// Configuration modules
mod config;
mod config_loader;
mod button_tracker;
mod command_queue;
mod timer_callbacks;

use config::{OutputField, Profile, ActionType};
#[cfg(test)]
use config::{ButtonMapping, AxisMapping};
use config_loader::Configuration;
use button_tracker::ButtonTracker;
use command_queue::{CommandQueue, Command, Priority};

/// Tracks enable button state changes
struct EnableStateTracker {
    _was_enabled: bool,
}

/// Load configuration parameters from multiple sources
fn load_parameters(_node: &safe_drive::node::Node, logger: &Logger) -> Result<(String, String), DynError> {
    use std::env;
    
    // Default values
    let mut config_file = String::new();
    let mut profile = "teleop".to_string();
    
    // 1. Check environment variables first
    if let Ok(env_config) = env::var("JOY_ROUTER_CONFIG_FILE") {
        config_file = env_config;
        pr_info!(logger, "Config file from environment: {}", config_file);
    }
    
    if let Ok(env_profile) = env::var("JOY_ROUTER_PROFILE") {
        profile = env_profile;
        pr_info!(logger, "Profile from environment: {}", profile);
    }
    
    // 2. Check command line arguments
    let args: Vec<String> = env::args().collect();
    for (i, arg) in args.iter().enumerate() {
        if arg == "--config" && i + 1 < args.len() {
            config_file = args[i + 1].clone();
            pr_info!(logger, "Config file from command line: {}", config_file);
        } else if arg == "--profile" && i + 1 < args.len() {
            profile = args[i + 1].clone();
            pr_info!(logger, "Profile from command line: {}", profile);
        } else if arg.starts_with("--config=") {
            config_file = arg.strip_prefix("--config=").unwrap().to_string();
            pr_info!(logger, "Config file from command line: {}", config_file);
        } else if arg.starts_with("--profile=") {
            profile = arg.strip_prefix("--profile=").unwrap().to_string();
            pr_info!(logger, "Profile from command line: {}", profile);
        }
    }
    
    // 3. Use defaults if nothing specified
    if config_file.is_empty() {
        config_file = "config/default.yaml".to_string();
        pr_info!(logger, "Using default config file: {}", config_file);
    }
    
    pr_info!(logger, "Final parameters - config_file: {}, profile: {}", config_file, profile);
    
    Ok((config_file, profile))
}

/// Converts a Joy message to a Twist message based on the given profile
fn joy_to_twist(joy: &Joy, profile: &Profile) -> Result<Twist, String> {
    let mut twist = Twist::new().ok_or_else(|| "Failed to create Twist".to_string())?;

    // Apply axis mappings
    for mapping in &profile.axis_mappings {
        // Check if the axis index is valid
        if mapping.joy_axis >= joy.axes.len() {
            continue; // Skip invalid indices gracefully
        }

        let value = joy.axes.as_slice()[mapping.joy_axis] as f64;
        let converted_value = mapping.apply(value);

        // Set the appropriate field in the Twist message
        match mapping.output_field {
            OutputField::LinearX => twist.linear.x = converted_value,
            OutputField::LinearY => twist.linear.y = converted_value,
            OutputField::LinearZ => twist.linear.z = converted_value,
            OutputField::AngularX => twist.angular.x = converted_value,
            OutputField::AngularY => twist.angular.y = converted_value,
            OutputField::AngularZ => twist.angular.z = converted_value,
        }
    }

    Ok(twist)
}

/// Checks if the enable button is pressed (if configured)
fn is_enabled(_joy: &Joy, profile: &Profile, button_tracker: &ButtonTracker) -> bool {
    // Check single enable button
    if let Some(button_idx) = profile.enable_button {
        if button_tracker.is_pressed(button_idx) {
            return true;
        }
    }
    
    // Check multiple enable buttons (OR logic)
    if let Some(ref buttons) = profile.enable_buttons {
        if button_tracker.is_any_pressed(buttons) {
            return true;
        }
    }
    
    // If no enable buttons configured, always enabled
    profile.enable_button.is_none() && profile.enable_buttons.is_none()
}

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("joy_msg_router", None, Default::default())?;
    let logger = Logger::new("joy_msg_router");

    // Load configuration from parameters
    let (config_path, profile_name) = load_parameters(&node, &logger)?;
    let configuration = Configuration::from_file(&config_path)?;
    let profile = configuration.get_profile(&profile_name).unwrap_or_else(|| {
        pr_info!(logger, "Failed to load profile '{}'. Using default profile.", profile_name);
        configuration.get_default_profile()
    });

    // Create subscriber and publisher
    let joy_subscriber = node.create_subscriber::<Joy>("joy", None)?;
    let twist_publisher = node.create_publisher::<Twist>("cmd_vel", None)?;

    pr_info!(logger, "Joy message router node started");
    
    // Log profile details
    if let Some(button) = profile.enable_button {
        pr_info!(logger, "Enable button: {}", button);
    }
    pr_info!(logger, "Axis mappings: {} configured", profile.axis_mappings.len());
    pr_info!(logger, "Button mappings: {} configured", profile.button_mappings.len());

    // Create a button tracker
    let mut button_tracker = ButtonTracker::new();
    let _enable_state = EnableStateTracker { _was_enabled: false };
    
    // Create command queue for decoupled processing
    let command_queue = Arc::new(CommandQueue::new());
    let queue_sender = command_queue.get_sender();
    
    // Track active button timers (button_id -> timer_active)
    let active_timers = Arc::new(Mutex::new(HashMap::<usize, bool>::new()));
    let active_timers_for_cb = Arc::clone(&active_timers);
    
    // Create a selector for handling callbacks
    let mut selector = ctx.create_selector()?;
    
    // Register timers for continuous button actions
    for button_mapping in profile.button_mappings.iter() {
        match &button_mapping.action {
            ActionType::PublishTwist { linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, once } if !once => {
                let button_id = button_mapping.button;
                let active_timers_clone = Arc::clone(&active_timers);
                let queue_sender_clone = queue_sender.clone();
                let twist_values = (*linear_x, *linear_y, *linear_z, *angular_x, *angular_y, *angular_z);
                
                selector.add_wall_timer(
                    &format!("button_{}_twist", button_id),
                    profile.timer_config.continuous_interval(),
                    Box::new(move || {
                        let timers = active_timers_clone.lock().unwrap();
                        if timers.get(&button_id).copied().unwrap_or(false) {
                            if let Some(mut twist) = Twist::new() {
                                twist.linear.x = twist_values.0;
                                twist.linear.y = twist_values.1;
                                twist.linear.z = twist_values.2;
                                twist.angular.x = twist_values.3;
                                twist.angular.y = twist_values.4;
                                twist.angular.z = twist_values.5;
                                
                                let _ = queue_sender_clone.send(command_queue::PrioritizedCommand {
                                    command: Command::PublishTwist(twist),
                                    priority: Priority::Normal,
                                });
                            }
                        }
                    }),
                );
            }
            ActionType::CallService { service_name, service_type, once } if !once => {
                let button_id = button_mapping.button;
                let active_timers_clone = Arc::clone(&active_timers);
                let queue_sender_clone = queue_sender.clone();
                let service_name_clone = service_name.clone();
                let service_type_clone = service_type.clone();
                
                selector.add_wall_timer(
                    &format!("button_{}_service", button_id),
                    profile.timer_config.continuous_interval(),
                    Box::new(move || {
                        let timers = active_timers_clone.lock().unwrap();
                        if timers.get(&button_id).copied().unwrap_or(false) {
                            let _ = queue_sender_clone.send(command_queue::PrioritizedCommand {
                                command: Command::CallService {
                                    service_name: service_name_clone.clone(),
                                    service_type: service_type_clone.clone(),
                                },
                                priority: Priority::Normal,
                            });
                        }
                    }),
                );
            }
            _ => {} // No timer needed for one-shot actions or NoAction
        }
    }

    selector.add_subscriber(
        joy_subscriber,
        Box::new(move |msg| {
            pr_debug!(logger, "Received Joy message");

            // Update button tracker
            button_tracker.update(msg.buttons.as_slice());

            // Check if output is enabled
            if !is_enabled(&msg, &profile, &button_tracker) {
                pr_debug!(logger, "Output disabled (enable button not pressed)");
                return;
            }

            // Process button actions first (these can override axis-based movement)
            let button_twist_override = None;
            for button_mapping in &profile.button_mappings {
                if button_mapping.button < msg.buttons.len() {
                    let _button_pressed = button_tracker.is_pressed(button_mapping.button);
                    let just_pressed = button_tracker.just_pressed(button_mapping.button);
                    let just_released = button_tracker.just_released(button_mapping.button);
                    
                    match &button_mapping.action {
                        ActionType::PublishTwist { linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, once } => {
                            if *once {
                                // One-shot action on button press
                                if just_pressed {
                                    if let Some(mut action_twist) = Twist::new() {
                                        action_twist.linear.x = *linear_x;
                                        action_twist.linear.y = *linear_y;
                                        action_twist.linear.z = *linear_z;
                                        action_twist.angular.x = *angular_x;
                                        action_twist.angular.y = *angular_y;
                                        action_twist.angular.z = *angular_z;
                                        
                                        pr_info!(logger, "Button {} pressed - publishing one-shot twist", button_mapping.button);
                                        
                                        if let Err(e) = queue_sender.send(command_queue::PrioritizedCommand {
                                            command: Command::PublishTwist(action_twist),
                                            priority: Priority::Normal,
                                        }) {
                                            pr_info!(logger, "Failed to enqueue one-shot Twist command: {:?}", e);
                                        }
                                    }
                                }
                            } else {
                                // Continuous action while button is held
                                if just_pressed {
                                    pr_info!(logger, "Button {} pressed - starting continuous twist", button_mapping.button);
                                    
                                    // Enable timer for this button
                                    let mut timers = active_timers_for_cb.lock().unwrap();
                                    timers.insert(button_mapping.button, true);
                                } else if just_released {
                                    pr_info!(logger, "Button {} released - stopping continuous twist", button_mapping.button);
                                    
                                    // Disable timer for this button
                                    let mut timers = active_timers_for_cb.lock().unwrap();
                                    timers.insert(button_mapping.button, false);
                                }
                            }
                        }
                        ActionType::CallService { service_name, service_type, once } => {
                            if *once {
                                // One-shot service call on button press
                                if just_pressed {
                                    pr_info!(logger, "Button {} pressed - calling service once: {}", 
                                            button_mapping.button, service_name);
                                    
                                    if let Err(e) = queue_sender.send(command_queue::PrioritizedCommand {
                                        command: Command::CallService {
                                            service_name: service_name.clone(),
                                            service_type: service_type.clone(),
                                        },
                                        priority: Priority::High, // Service calls get higher priority
                                }) {
                                    pr_info!(logger, "Failed to enqueue service call: {:?}", e);
                                }
                            }
                            } else {
                                // Continuous service calls while button is held
                                if just_pressed {
                                    pr_info!(logger, "Button {} pressed - starting continuous service calls: {}", 
                                            button_mapping.button, service_name);
                                    
                                    // Enable timer for this button
                                    let mut timers = active_timers_for_cb.lock().unwrap();
                                    timers.insert(button_mapping.button, true);
                                } else if just_released {
                                    pr_info!(logger, "Button {} released - stopping continuous service calls: {}", 
                                            button_mapping.button, service_name);
                                    
                                    // Disable timer for this button
                                    let mut timers = active_timers_for_cb.lock().unwrap();
                                    timers.insert(button_mapping.button, false);
                                }
                            }
                        }
                        ActionType::NoAction => {
                            if just_pressed {
                                pr_info!(logger, "Button {} pressed - stop action", button_mapping.button);
                                
                                // Send stop command
                                if let Err(e) = queue_sender.send(command_queue::PrioritizedCommand {
                                    command: Command::Stop,
                                    priority: Priority::High,
                                }) {
                                    pr_info!(logger, "Failed to enqueue stop command: {:?}", e);
                                }
                            }
                        }
                    }
                }
            }

            // Determine which Twist to publish
            let twist_to_publish = if let Some(button_twist) = button_twist_override {
                // Button action takes precedence
                button_twist
            } else {
                // Use axis-based conversion
                match joy_to_twist(&msg, &profile) {
                    Ok(twist) => twist,
                    Err(e) => {
                        pr_info!(logger, "Failed to convert Joy to Twist: {}", e);
                        return;
                    }
                }
            };

            // Log non-zero values
            if twist_to_publish.linear.x != 0.0 || twist_to_publish.angular.z != 0.0 {
                pr_debug!(
                    logger,
                    "Publishing Twist: linear.x={:.3}, angular.z={:.3}",
                    twist_to_publish.linear.x,
                    twist_to_publish.angular.z
                );
            }

            // Enqueue the Twist command instead of directly publishing
            if let Err(e) = queue_sender.send(command_queue::PrioritizedCommand {
                command: Command::PublishTwist(twist_to_publish),
                priority: Priority::Normal,
            }) {
                pr_info!(logger, "Failed to enqueue Twist command: {:?}", e);
            }
        }),
    );

    // Clone command queue for the processing loop
    let queue_for_processing = Arc::clone(&command_queue);
    
    // Create a separate logger for the command processing loop
    let process_logger = Logger::new("joy_msg_router_cmd_processor");
    
    // Spin the selector and process commands
    loop {
        // Wait for events with a timeout to allow command processing
        selector.wait_timeout(std::time::Duration::from_millis(10))?;
        
        // Process pending commands
        queue_for_processing.process_pending(|cmd| {
            match cmd.command {
                Command::PublishTwist(twist) => {
                    if let Err(e) = twist_publisher.send(&twist) {
                        pr_info!(process_logger, "Failed to publish Twist message: {:?}", e);
                    }
                }
                Command::CallService { service_name, service_type } => {
                    pr_info!(process_logger, "Would call service: {} (type: {})", service_name, service_type);
                    // TODO: Implement actual service client when safe_drive supports it
                }
                Command::Stop => {
                    // Send zero twist
                    if let Some(zero_twist) = Twist::new() {
                        if let Err(e) = twist_publisher.send(&zero_twist) {
                            pr_info!(process_logger, "Failed to publish stop command: {:?}", e);
                        }
                    }
                }
            }
            Ok(())
        })?;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::button_tracker::ButtonTracker;

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

    fn create_test_profile() -> Profile {
        let mut profile = Profile::new("test".to_string());

        // Map axis 0 to linear.x
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 0,
            output_field: OutputField::LinearX,
            scale: 2.0,
            offset: 0.0,
            deadzone: 0.1,
        });

        // Map axis 1 to angular.z
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 1,
            output_field: OutputField::AngularZ,
            scale: -1.0,
            offset: 0.5,
            deadzone: 0.05,
        });

        profile
    }

    #[test]
    fn test_joy_to_twist_basic() {
        let joy = create_test_joy(vec![0.5, 0.3], vec![]);
        let profile = create_test_profile();

        let twist = joy_to_twist(&joy, &profile).unwrap();

        assert!((twist.linear.x - 1.0).abs() < 0.001); // 0.5 * 2.0
        assert!((twist.angular.z - 0.2).abs() < 0.001); // 0.3 * -1.0 + 0.5
    }

    #[test]
    fn test_joy_to_twist_deadzone() {
        let joy = create_test_joy(vec![0.05, 0.04], vec![]); // Below deadzones
        let profile = create_test_profile();

        let twist = joy_to_twist(&joy, &profile).unwrap();

        assert_eq!(twist.linear.x, 0.0); // Below 0.1 deadzone
        assert_eq!(twist.angular.z, 0.0); // Below 0.05 deadzone, should be 0 (offset not applied when in deadzone)
    }

    #[test]
    fn test_joy_to_twist_missing_axes() {
        let joy = create_test_joy(vec![0.5], vec![]); // Only one axis
        let profile = create_test_profile();

        let twist = joy_to_twist(&joy, &profile).unwrap();

        assert_eq!(twist.linear.x, 1.0); // 0.5 * 2.0
        assert_eq!(twist.angular.z, 0.0); // Axis 1 missing, default to 0
    }

    #[test]
    fn test_is_enabled_no_button() {
        let joy = create_test_joy(vec![], vec![0, 1, 0]);
        let profile = Profile::new("test".to_string()); // No enable button
        let mut tracker = ButtonTracker::new();
        tracker.update(&[0, 1, 0]);

        assert!(is_enabled(&joy, &profile, &tracker));
    }

    #[test]
    fn test_is_enabled_with_button() {
        let joy = create_test_joy(vec![], vec![0, 1, 0]);
        let mut profile = Profile::new("test".to_string());
        let mut tracker = ButtonTracker::new();
        tracker.update(&[0, 1, 0]);

        profile.enable_button = Some(1);
        assert!(is_enabled(&joy, &profile, &tracker)); // Button 1 is pressed

        profile.enable_button = Some(0);
        assert!(!is_enabled(&joy, &profile, &tracker)); // Button 0 is not pressed

        profile.enable_button = Some(5);
        assert!(!is_enabled(&joy, &profile, &tracker)); // Button 5 doesn't exist
    }
    
    #[test]
    fn test_is_enabled_multiple_buttons() {
        let joy = create_test_joy(vec![], vec![0, 0, 1, 0]);
        let mut profile = Profile::new("test".to_string());
        let mut tracker = ButtonTracker::new();
        tracker.update(&[0, 0, 1, 0]);

        // Test multiple enable buttons (OR logic)
        profile.enable_buttons = Some(vec![0, 1, 2]);
        assert!(is_enabled(&joy, &profile, &tracker)); // Button 2 is pressed

        profile.enable_buttons = Some(vec![0, 1]);
        assert!(!is_enabled(&joy, &profile, &tracker)); // Neither 0 nor 1 pressed
    }
    
    #[test]
    fn test_joy_to_twist_boundary_values() {
        let joy = create_test_joy(vec![1.0, -1.0, 0.0], vec![]);
        let mut profile = create_test_profile();
        
        // Add a third axis mapping
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 2,
            output_field: OutputField::LinearY,
            scale: 1.5,
            offset: 0.0,
            deadzone: 0.05,
        });
        
        let twist = joy_to_twist(&joy, &profile).unwrap();
        
        assert_eq!(twist.linear.x, 2.0); // 1.0 * 2.0 (max positive)
        assert_eq!(twist.angular.z, 1.5); // -1.0 * -1.0 + 0.5 = 1.0 + 0.5 = 1.5
        assert_eq!(twist.linear.y, 0.0); // 0.0 * 1.5 (exact zero)
    }
    
    #[test]
    fn test_joy_to_twist_all_axes_below_deadzone() {
        let joy = create_test_joy(vec![0.05, 0.04, 0.01], vec![]);
        let mut profile = Profile::new("test".to_string());
        
        // All axes with deadzone higher than input values
        for i in 0..3 {
            profile.axis_mappings.push(AxisMapping {
                joy_axis: i,
                output_field: match i {
                    0 => OutputField::LinearX,
                    1 => OutputField::LinearY,
                    _ => OutputField::LinearZ,
                },
                scale: 1.0,
                offset: 0.0,
                deadzone: 0.1,
            });
        }
        
        let twist = joy_to_twist(&joy, &profile).unwrap();
        
        // All values should be zero due to deadzone
        assert_eq!(twist.linear.x, 0.0);
        assert_eq!(twist.linear.y, 0.0);
        assert_eq!(twist.linear.z, 0.0);
    }
    
    #[test]
    fn test_joy_to_twist_with_large_offset() {
        let joy = create_test_joy(vec![0.0], vec![]);
        let mut profile = Profile::new("test".to_string());
        
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 0,
            output_field: OutputField::AngularZ,
            scale: 1.0,
            offset: 2.5, // Large offset
            deadzone: 0.01,
        });
        
        let twist = joy_to_twist(&joy, &profile).unwrap();
        
        // Even with zero input, offset should be applied
        assert_eq!(twist.angular.z, 0.0); // 0.0 is within deadzone, so result is 0
    }
    
    #[test]
    fn test_joy_to_twist_negative_scale() {
        let joy = create_test_joy(vec![0.5, -0.5], vec![]);
        let mut profile = Profile::new("test".to_string());
        
        // Negative scale inverts the axis
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 0,
            output_field: OutputField::LinearX,
            scale: -2.0,
            offset: 0.0,
            deadzone: 0.1,
        });
        
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 1,
            output_field: OutputField::AngularZ,
            scale: -1.0,
            offset: 0.0,
            deadzone: 0.1,
        });
        
        let twist = joy_to_twist(&joy, &profile).unwrap();
        
        assert_eq!(twist.linear.x, -1.0); // 0.5 * -2.0
        assert_eq!(twist.angular.z, 0.5); // -0.5 * -1.0
    }
    
    #[test]
    fn test_is_enabled_both_single_and_multiple() {
        let joy = create_test_joy(vec![], vec![1, 0, 1, 0]);
        let mut profile = Profile::new("test".to_string());
        let mut tracker = ButtonTracker::new();
        tracker.update(&[1, 0, 1, 0]);

        // Test when both single and multiple buttons are configured
        profile.enable_button = Some(1); // Not pressed
        profile.enable_buttons = Some(vec![2, 3]); // Button 2 is pressed
        
        // Should be enabled because button 2 is pressed (OR logic between single and multiple)
        assert!(is_enabled(&joy, &profile, &tracker));
    }
    
    #[test]
    fn test_profile_with_duplicate_output_fields() {
        let joy = create_test_joy(vec![0.5, 0.3], vec![]);
        let mut profile = Profile::new("test".to_string());
        
        // Two mappings to the same output field - last one should win
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 0,
            output_field: OutputField::LinearX,
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.1,
        });
        
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 1,
            output_field: OutputField::LinearX,
            scale: 2.0,
            offset: 0.0,
            deadzone: 0.1,
        });
        
        let twist = joy_to_twist(&joy, &profile).unwrap();
        
        // Last mapping (axis 1) should override
        assert!((twist.linear.x - 0.6).abs() < 0.0001); // 0.3 * 2.0 with tolerance
    }

    #[test]
    fn test_button_action_publish_twist() {
        let mut profile = Profile::new("test".to_string());
        
        // Add a button mapping that publishes a fixed twist
        profile.button_mappings.push(ButtonMapping {
            button: 0,
            action: ActionType::PublishTwist {
                linear_x: 1.0,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_x: 0.0,
                angular_y: 0.0,
                angular_z: 0.5,
                once: false,
            },
        });
        
        // Test that the button action configuration is correct
        assert_eq!(profile.button_mappings.len(), 1);
        match &profile.button_mappings[0].action {
            ActionType::PublishTwist { linear_x, angular_z, once: _, .. } => {
                assert_eq!(*linear_x, 1.0);
                assert_eq!(*angular_z, 0.5);
            }
            _ => panic!("Expected PublishTwist action"),
        }
    }

    #[test]
    fn test_button_action_call_service() {
        let mut profile = Profile::new("test".to_string());
        
        // Add a button mapping that calls a service
        profile.button_mappings.push(ButtonMapping {
            button: 1,
            action: ActionType::CallService {
                service_name: "/test_service".to_string(),
                service_type: "std_srvs/srv/Trigger".to_string(),
                once: true,
            },
        });
        
        // Test that the service action configuration is correct
        assert_eq!(profile.button_mappings.len(), 1);
        match &profile.button_mappings[0].action {
            ActionType::CallService { service_name, service_type, once: _ } => {
                assert_eq!(service_name, "/test_service");
                assert_eq!(service_type, "std_srvs/srv/Trigger");
            }
            _ => panic!("Expected CallService action"),
        }
    }

    #[test]
    fn test_parameter_loading() {
        use std::env;
        
        // Test default values
        let logger = safe_drive::logger::Logger::new("test");
        let context = safe_drive::context::Context::new().unwrap();
        let node = context.create_node("test_node", None, Default::default()).unwrap();
        
        // Save original environment
        let original_config = env::var("JOY_ROUTER_CONFIG_FILE").ok();
        let original_profile = env::var("JOY_ROUTER_PROFILE").ok();
        
        // Clear environment variables
        env::remove_var("JOY_ROUTER_CONFIG_FILE");
        env::remove_var("JOY_ROUTER_PROFILE");
        
        // Test defaults
        let (config_file, profile) = load_parameters(&node, &logger).unwrap();
        assert_eq!(config_file, "config/default.yaml");
        assert_eq!(profile, "teleop");
        
        // Test environment variables
        env::set_var("JOY_ROUTER_CONFIG_FILE", "/custom/config.yaml");
        env::set_var("JOY_ROUTER_PROFILE", "drone");
        
        let (config_file, profile) = load_parameters(&node, &logger).unwrap();
        assert_eq!(config_file, "/custom/config.yaml");
        assert_eq!(profile, "drone");
        
        // Restore original environment
        if let Some(config) = original_config {
            env::set_var("JOY_ROUTER_CONFIG_FILE", config);
        } else {
            env::remove_var("JOY_ROUTER_CONFIG_FILE");
        }
        if let Some(profile) = original_profile {
            env::set_var("JOY_ROUTER_PROFILE", profile);
        } else {
            env::remove_var("JOY_ROUTER_PROFILE");
        }
    }
}
