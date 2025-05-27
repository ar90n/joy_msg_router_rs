use safe_drive::{context::Context, error::DynError, logger::Logger, pr_debug, pr_info};

// Import ROS message types
use geometry_msgs::msg::Twist;
use sensor_msgs::msg::Joy;

// Configuration module
mod config;

use config::{AxisMapping, OutputField, Profile};

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
fn is_enabled(joy: &Joy, profile: &Profile) -> bool {
    match profile.enable_button {
        Some(button_idx) => {
            // Check if button index is valid and pressed
            button_idx < joy.buttons.len() && joy.buttons.as_slice()[button_idx] != 0
        }
        None => true, // Always enabled if no enable button is configured
    }
}

fn main() -> Result<(), DynError> {
    // Create a context.
    let ctx = Context::new()?;

    // Create a node.
    let node = ctx.create_node("joy_msg_router", None, Default::default())?;

    // Create a logger.
    let logger = Logger::new("joy_msg_router");

    // Create a subscriber for Joy messages
    let subscriber = node.create_subscriber::<Joy>("joy", None)?;

    // Create a publisher for Twist messages
    let twist_publisher = node.create_publisher::<Twist>("cmd_vel", None)?;

    // Create a default profile for testing
    // TODO: Load from configuration file
    let mut profile = Profile::new("default".to_string());

    // Add default axis mappings (common joystick configuration)
    // Left stick Y-axis (1) -> linear.x (forward/backward)
    profile.axis_mappings.push(AxisMapping {
        joy_axis: 1,
        output_field: OutputField::LinearX,
        scale: 0.5, // Max 0.5 m/s
        offset: 0.0,
        deadzone: 0.1,
    });

    // Right stick X-axis (3) -> angular.z (rotation)
    profile.axis_mappings.push(AxisMapping {
        joy_axis: 3,
        output_field: OutputField::AngularZ,
        scale: 1.0, // Max 1.0 rad/s
        offset: 0.0,
        deadzone: 0.1,
    });

    pr_info!(logger, "Joy message router node started");
    pr_info!(logger, "Listening for Joy messages on /joy topic");
    pr_info!(logger, "Publishing Twist messages to /cmd_vel topic");
    pr_info!(
        logger,
        "Using default profile with axis 1->linear.x, axis 3->angular.z"
    );

    // Create a selector for handling callbacks
    let mut selector = ctx.create_selector()?;

    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            pr_debug!(logger, "Received Joy message");

            // Check if output is enabled
            if !is_enabled(&msg, &profile) {
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

#[cfg(test)]
mod tests {
    use super::*;

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

        assert!(is_enabled(&joy, &profile));
    }

    #[test]
    fn test_is_enabled_with_button() {
        let joy = create_test_joy(vec![], vec![0, 1, 0]);
        let mut profile = Profile::new("test".to_string());

        profile.enable_button = Some(1);
        assert!(is_enabled(&joy, &profile)); // Button 1 is pressed

        profile.enable_button = Some(0);
        assert!(!is_enabled(&joy, &profile)); // Button 0 is not pressed

        profile.enable_button = Some(5);
        assert!(!is_enabled(&joy, &profile)); // Button 5 doesn't exist
    }
}
