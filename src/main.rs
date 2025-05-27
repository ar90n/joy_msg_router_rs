use safe_drive::{context::Context, error::DynError, logger::Logger, pr_debug, pr_info};
use std::sync::Arc;

// Import ROS message types
use geometry_msgs::msg::Twist;
use sensor_msgs::msg::Joy;

// Configuration modules
mod config;
mod config_loader;
mod button_tracker;

use config::{AxisMapping, OutputField, Profile, ActionType};
use config_loader::Configuration;
use button_tracker::ButtonTracker;

/// Tracks enable button state changes
struct EnableStateTracker {
    was_enabled: bool,
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
    // Create a context.
    let ctx = Context::new()?;

    // Create a node.
    let node = ctx.create_node("joy_msg_router", None, Default::default())?;

    // Create a logger.
    let logger = Logger::new("joy_msg_router");

    // Load configuration file
    // TODO: Get config file path from ROS2 parameter or command line
    let config_path = "config/default.yaml";
    let configuration = match Configuration::from_file(config_path) {
        Ok(config) => {
            pr_info!(logger, "Loaded configuration from: {}", config_path);
            config
        }
        Err(e) => {
            pr_info!(logger, "Failed to load config file: {}. Using hardcoded defaults.", e);
            // Fall back to hardcoded configuration
            return run_with_hardcoded_config(ctx, node, logger);
        }
    };

    // Create a subscriber for Joy messages
    let subscriber = node.create_subscriber::<Joy>("joy", None)?;

    // Create a publisher for Twist messages
    let twist_publisher = node.create_publisher::<Twist>("cmd_vel", None)?;

    // Get the default profile from configuration
    let profile = configuration.get_default_profile();

    pr_info!(logger, "Joy message router node started");
    pr_info!(logger, "Listening for Joy messages on /joy topic");
    pr_info!(logger, "Publishing Twist messages to /cmd_vel topic");
    pr_info!(logger, "Using profile '{}' from configuration", profile.name);
    
    // Log profile details
    if let Some(button) = profile.enable_button {
        pr_info!(logger, "Enable button: {}", button);
    }
    pr_info!(logger, "Axis mappings: {} configured", profile.axis_mappings.len());
    pr_info!(logger, "Button mappings: {} configured", profile.button_mappings.len());

    // Create a button tracker
    let mut button_tracker = ButtonTracker::new();
    let mut enable_state = EnableStateTracker { was_enabled: false };

    // Create a selector for handling callbacks
    let mut selector = ctx.create_selector()?;

    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            pr_debug!(logger, "Received Joy message");

            // Update button tracker
            button_tracker.update(msg.buttons.as_slice());

            // Check if output is enabled
            if !is_enabled(&msg, &profile, &button_tracker) {
                pr_debug!(logger, "Output disabled (enable button not pressed)");
                return;
            }

            // Convert Joy to Twist using the profile
            match joy_to_twist(&msg, &profile) {
                Ok(mut twist_msg) => {
                    // Process button actions
                    for button_mapping in &profile.button_mappings {
                        if button_tracker.is_pressed(button_mapping.button) {
                            match &button_mapping.action {
                                ActionType::PublishTwist { linear_x, linear_y, linear_z, angular_x, angular_y, angular_z } => {
                                    // Override twist values with button action
                                    twist_msg.linear.x = *linear_x;
                                    twist_msg.linear.y = *linear_y;
                                    twist_msg.linear.z = *linear_z;
                                    twist_msg.angular.x = *angular_x;
                                    twist_msg.angular.y = *angular_y;
                                    twist_msg.angular.z = *angular_z;
                                    
                                    if button_tracker.just_pressed(button_mapping.button) {
                                        pr_info!(logger, "Button {} pressed - publishing configured twist", button_mapping.button);
                                    }
                                }
                                ActionType::CallService { service_name, service_type: _ } => {
                                    if button_tracker.just_pressed(button_mapping.button) {
                                        pr_info!(logger, "Button {} pressed - would call service {} (not implemented)", 
                                                button_mapping.button, service_name);
                                    }
                                }
                            }
                        }
                    }

                    // Log non-zero values
                    if twist_msg.linear.x != 0.0 || twist_msg.angular.z != 0.0 {
                        pr_debug!(
                            logger,
                            "Publishing Twist: linear.x={:.3}, angular.z={:.3}",
                            twist_msg.linear.x,
                            twist_msg.angular.z
                        );
                    }

                    // Publish the Twist message
                    if let Err(e) = twist_publisher.send(&twist_msg) {
                        pr_info!(logger, "Failed to publish Twist message: {:?}", e);
                    }
                }
                Err(e) => {
                    pr_info!(logger, "Failed to convert Joy to Twist: {}", e);
                }
            }
        }),
    );

    // Spin the selector
    loop {
        selector.wait()?;
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
}

/// Fallback function when configuration file cannot be loaded
fn run_with_hardcoded_config(
    ctx: Arc<Context>,
    node: Arc<safe_drive::node::Node>,
    logger: Logger,
) -> Result<(), DynError> {
    // Create a subscriber for Joy messages
    let subscriber = node.create_subscriber::<Joy>("joy", None)?;

    // Create a publisher for Twist messages
    let twist_publisher = node.create_publisher::<Twist>("cmd_vel", None)?;

    // Create a profile with default values
    let mut profile = Profile::new("hardcoded_default".to_string());

    // Default parameter values
    let linear_x_axis_val = 1_usize;  // axis 1
    let linear_x_scale_val = 0.5;     // 0.5 m/s max
    let angular_z_axis_val = 3_usize; // axis 3
    let angular_z_scale_val = 1.0;    // 1.0 rad/s max
    let deadzone_val = 0.1;           // 0.1 deadzone
    let enable_button_val = -1_i64;   // -1 (always enabled)

    // Configure enable button
    if enable_button_val >= 0 {
        profile.enable_button = Some(enable_button_val as usize);
    }

    // Add linear.x axis mapping
    profile.axis_mappings.push(AxisMapping {
        joy_axis: linear_x_axis_val,
        output_field: OutputField::LinearX,
        scale: linear_x_scale_val,
        offset: 0.0,
        deadzone: deadzone_val,
    });

    // Add angular.z axis mapping
    profile.axis_mappings.push(AxisMapping {
        joy_axis: angular_z_axis_val,
        output_field: OutputField::AngularZ,
        scale: angular_z_scale_val,
        offset: 0.0,
        deadzone: deadzone_val,
    });

    pr_info!(logger, "Joy message router node started");
    pr_info!(logger, "Listening for Joy messages on /joy topic");
    pr_info!(logger, "Publishing Twist messages to /cmd_vel topic");
    pr_info!(logger, "Using hardcoded default configuration");

    // Create a button tracker
    let mut button_tracker = ButtonTracker::new();

    // Create a selector for handling callbacks
    let mut selector = ctx.create_selector()?;

    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            pr_debug!(logger, "Received Joy message");

            // Update button tracker
            button_tracker.update(msg.buttons.as_slice());

            // Check if output is enabled
            if !is_enabled(&msg, &profile, &button_tracker) {
                pr_debug!(logger, "Output disabled (enable button not pressed)");
                return;
            }

            // Convert Joy to Twist using the profile
            match joy_to_twist(&msg, &profile) {
                Ok(twist_msg) => {
                    // Log non-zero values
                    if twist_msg.linear.x != 0.0 || twist_msg.angular.z != 0.0 {
                        pr_debug!(
                            logger,
                            "Publishing Twist: linear.x={:.3}, angular.z={:.3}",
                            twist_msg.linear.x,
                            twist_msg.angular.z
                        );
                    }

                    // Publish the Twist message
                    if let Err(e) = twist_publisher.send(&twist_msg) {
                        pr_info!(logger, "Failed to publish Twist message: {:?}", e);
                    }
                }
                Err(e) => {
                    pr_info!(logger, "Failed to convert Joy to Twist: {}", e);
                }
            }
        }),
    );

    // Spin the selector
    loop {
        selector.wait()?;
    }
}
