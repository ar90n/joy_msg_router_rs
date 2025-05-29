use safe_drive::{
    context::Context as SafeDriveContext, logger::Logger, pr_debug, pr_error, pr_info,
};
use std::sync::{Arc, Mutex};

use geometry_msgs::msg::Twist;
use sensor_msgs::msg::Joy;
use std_msgs::msg::{Bool, Float64, Int32, String as StringMsg};

mod config;
mod joy_msg_tracker;
mod logging;
mod profile;
mod publishers;

use anyhow::{Result, anyhow};
use config::{ActionType, InputMapping, InputSource, OutputField};
use joy_msg_tracker::JoyMsgTracker;
use logging::{log_error, LogContext};
use profile::{is_enabled, load_profile_from_params};
use publishers::Publishers;
use anyhow::Context;

fn main() -> Result<()> {
    main_impl()
}

/// Process all input mappings and handle their actions
fn process_input_mappings(
    tracker: &JoyMsgTracker,
    input_mappings: &[InputMapping],
    publishers: &Publishers,
    logger: &Logger,
) -> Result<()> {
    let mut twist_accumulator = Twist::new()
        .ok_or_else(|| anyhow!("Failed to create Twist message"))?;
    let mut has_twist_contribution = false;

    for mapping in input_mappings {
        let (input_value, is_active, just_activated) = match mapping.source {
            InputSource::Axis(idx) => {
                let value = tracker.get_axis(idx).unwrap_or(0.0) as f64;
                let processed = mapping.process_value(value);
                (processed, processed.abs() > 0.0, false)
            }
            InputSource::Button(idx) => {
                let pressed = tracker.is_pressed(idx);
                let just_pressed = tracker.just_pressed(idx);
                let value = if pressed { 1.0 } else { 0.0 };
                (value, pressed, just_pressed)
            }
        };

        match &mapping.action {
            ActionType::PublishTwistField { field } => {
                if is_active {
                    match field {
                        OutputField::LinearX => twist_accumulator.linear.x += input_value,
                        OutputField::LinearY => twist_accumulator.linear.y += input_value,
                        OutputField::LinearZ => twist_accumulator.linear.z += input_value,
                        OutputField::AngularX => twist_accumulator.angular.x += input_value,
                        OutputField::AngularY => twist_accumulator.angular.y += input_value,
                        OutputField::AngularZ => twist_accumulator.angular.z += input_value,
                    }
                    has_twist_contribution = true;
                }
            }
            ActionType::PublishBool { topic, value, once } => {
                if (*once && just_activated) || (!*once && is_active) {
                    if let Some(mut bool_msg) = Bool::new() {
                        bool_msg.data = *value;
                        if let Some(publisher) = publishers.bool_publishers.get(topic) {
                            if let Err(e) = publisher.send(&bool_msg) {
                                log_error(
                                    logger,
                                    LogContext {
                                        module: "main",
                                        function: "process_input_mappings",
                                        details: Some("publish_bool"),
                                    },
                                    &anyhow!(e.to_string()),
                                );
                            }
                        }
                    }
                }
            }
            ActionType::PublishInt32 { topic, value, once } => {
                if (*once && just_activated) || (!*once && is_active) {
                    if let Some(mut int_msg) = Int32::new() {
                        int_msg.data = *value;
                        if let Some(publisher) = publishers.int32_publishers.get(topic) {
                            if let Err(e) = publisher.send(&int_msg) {
                                log_error(
                                    logger,
                                    LogContext {
                                        module: "main",
                                        function: "process_input_mappings",
                                        details: Some("publish_int32"),
                                    },
                                    &anyhow!(e.to_string()),
                                );
                            }
                        }
                    }
                }
            }
            ActionType::PublishFloat64 { topic, value, once } => {
                if (*once && just_activated) || (!*once && is_active) {
                    if let Some(mut float_msg) = Float64::new() {
                        float_msg.data = *value;
                        if let Some(publisher) = publishers.float64_publishers.get(topic) {
                            if let Err(e) = publisher.send(&float_msg) {
                                log_error(
                                    logger,
                                    LogContext {
                                        module: "main",
                                        function: "process_input_mappings",
                                        details: Some("publish_float64"),
                                    },
                                    &anyhow!(e.to_string()),
                                );
                            }
                        }
                    }
                }
            }
            ActionType::PublishString { topic, value, once } => {
                if (*once && just_activated) || (!*once && is_active) {
                    if let Some(mut str_msg) = StringMsg::new() {
                        if let Some(data_ros) = safe_drive::msg::RosString::<0>::new(value) {
                            str_msg.data = data_ros;
                            if let Some(publisher) = publishers.string_publishers.get(topic) {
                                if let Err(e) = publisher.send(&str_msg) {
                                    log_error(
                                        logger,
                                        LogContext {
                                            module: "main",
                                            function: "process_input_mappings",
                                            details: Some("publish_string"),
                                        },
                                        &anyhow!(e.to_string()),
                                    );
                                }
                            }
                        }
                    }
                }
            }
            ActionType::CallService {
                service_name,
                service_type,
                once,
            } => {
                if (*once && just_activated) || (!*once && is_active) {
                    pr_info!(
                        logger,
                        "Would call service: {} (type: {})",
                        service_name, service_type
                    );
                }
            }
            ActionType::NoAction => {
                if just_activated {
                    if let Some(zero_twist) = Twist::new() {
                        if let Err(e) = publishers.twist_publisher.send(&zero_twist) {
                            log_error(
                                logger,
                                LogContext {
                                    module: "main",
                                    function: "process_input_mappings",
                                    details: Some("stop"),
                                },
                                &anyhow!(e.to_string()),
                            );
                        }
                    }
                }
            }
            _ => {}
        }
    }

    if has_twist_contribution {
        if let Err(e) = publishers.twist_publisher.send(&twist_accumulator) {
            log_error(
                logger,
                LogContext {
                    module: "main",
                    function: "process_input_mappings",
                    details: Some("accumulated_twist"),
                },
                &anyhow!(e.to_string()),
            );
        }
    }

    Ok(())
}

fn main_impl() -> Result<()> {
    let logger = Logger::new("joy_msg_router");
    let ctx = SafeDriveContext::new()
        .map_err(|e| anyhow!("Failed to create ROS2 context: {:?}", e))?;
    let node = Arc::new(
        ctx.create_node("joy_msg_router", None, Default::default())
            .map_err(|e| anyhow!("Failed to create ROS2 node: {:?}", e))?,
    );
    let profile = node.create_parameter_server()
            .map_err(|e| anyhow!("Failed to create parameter server: {:?}", e))
            .and_then(|ps| load_profile_from_params(&ps))?;

    pr_info!(logger, "Joy message router node started");
    pr_info!(logger, "Active profile: {}", profile.name);

    let joy_subscriber = node
        .create_subscriber::<Joy>("joy", None)
        .map_err(|e| anyhow!("Failed to create joy subscriber: {:?}", e))?;

    let publishers = Arc::new(Publishers::from_profile(&node, &profile)
        .context("Failed to create publishers")?);

    let joy_tracker = Arc::new(Mutex::new(JoyMsgTracker::new()));
    let joy_tracker_sub = Arc::clone(&joy_tracker);

    let mut selector = ctx.create_selector()
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
            process_input_mappings(
                &tracker,
                &profile.input_mappings,
                &publishers,
                &logger,
            )?;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Profile, InputMapping, InputSource, ActionType, OutputField};
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

    fn create_test_profile() -> Profile {
        let mut profile = Profile::new("test".to_string());

        // Map axis 0 to linear.x
        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::PublishTwistField {
                field: OutputField::LinearX,
            },
            scale: 2.0,
            offset: 0.0,
            deadzone: 0.1,
        });

        // Map axis 1 to angular.z
        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(1),
            action: ActionType::PublishTwistField {
                field: OutputField::AngularZ,
            },
            scale: -1.0,
            offset: 0.5,
            deadzone: 0.05,
        });

        profile
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

        // Test multiple enable buttons (OR logic)
        profile.enable_buttons = Some(vec![0, 1, 2]);
        {
            let tracker_guard = tracker.lock().unwrap();
            assert!(is_enabled(&profile, &tracker_guard)); // Button 2 is pressed
        }

        profile.enable_buttons = Some(vec![0, 1]);
        {
            let tracker_guard = tracker.lock().unwrap();
            assert!(!is_enabled(&profile, &tracker_guard)); // Neither 0 nor 1 pressed
        }
    }
}