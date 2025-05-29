use safe_drive::{context::Context, error::DynError, logger::Logger, pr_debug, pr_info, pr_error, pr_warn};
use std::sync::{Arc, Mutex};

// Import ROS message types
use geometry_msgs::msg::Twist;
use sensor_msgs::msg::Joy;

// Configuration modules
mod config;
mod config_loader;
mod button_tracker;
mod command_queue;
mod timer_callbacks;
mod publishers;
mod error;
mod logging;

#[cfg(test)]
mod test_multiple_message_types;

use config::{OutputField, Profile, ActionType};
#[cfg(test)]
use config::{ButtonMapping, AxisMapping};
use config_loader::Configuration;
use button_tracker::ButtonTracker;
use command_queue::{CommandQueue, Command, Priority};
use publishers::Publishers;
use error::{JoyRouterError, JoyRouterResult, ErrorContext};
use logging::{log_command_result};

/// Tracks enable button state changes
struct EnableStateTracker {
    _was_enabled: bool,
}

/// Load configuration parameters from multiple sources
fn load_parameters(_node: &safe_drive::node::Node, logger: &Logger) -> JoyRouterResult<(String, String)> {
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
            config_file = arg.strip_prefix("--config=")
                .ok_or_else(|| JoyRouterError::ConfigError("Invalid --config= argument".to_string()))?
                .to_string();
            pr_info!(logger, "Config file from command line: {}", config_file);
        } else if arg.starts_with("--profile=") {
            profile = arg.strip_prefix("--profile=")
                .ok_or_else(|| JoyRouterError::ConfigError("Invalid --profile= argument".to_string()))?
                .to_string();
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

/// Converts joy axes to a Twist message based on the given profile
fn axes_to_twist(axes: &[f32], profile: &Profile) -> JoyRouterResult<Twist> {
    let mut twist = Twist::new()
        .ok_or_else(|| JoyRouterError::MessageError("Failed to create Twist message".to_string()))?;

    // Apply axis mappings
    for mapping in &profile.axis_mappings {
        // Check if the axis index is valid
        if mapping.joy_axis >= axes.len() {
            continue; // Skip invalid indices gracefully
        }

        let value = axes[mapping.joy_axis] as f64;
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

/// Converts a Joy message to a Twist message based on the given profile  
fn joy_to_twist(joy: &Joy, profile: &Profile) -> JoyRouterResult<Twist> {
    axes_to_twist(joy.axes.as_slice(), profile)
}

/// Checks if the enable button is pressed (if configured)
fn is_enabled(profile: &Profile, button_tracker: &std::sync::MutexGuard<ButtonTracker>) -> bool {
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
    // Use proper error handling throughout
    main_impl().map_err(|e| Box::new(e) as DynError)
}

fn main_impl() -> JoyRouterResult<()> {
    let ctx = Context::new()
        .context("Failed to create ROS2 context")?;
    let node = Arc::new(ctx.create_node("joy_msg_router", None, Default::default())
        .context("Failed to create ROS2 node")?);
    let logger = Logger::new("joy_msg_router");

    // Load configuration from parameters
    let (config_path, profile_name) = load_parameters(&node, &logger)?;
    let configuration = Configuration::from_file(&config_path)
        .map_err(|e| JoyRouterError::ConfigError(format!("Failed to load config file '{}': {}", config_path, e)))?;
    let profile = configuration.get_profile(&profile_name).unwrap_or_else(|| {
        log_warn!(&logger, "config_loader", "load_profile", 
            &format!("Failed to load profile '{}'. Using default profile.", profile_name));
        configuration.get_default_profile()
    });

    // Create subscriber
    let joy_subscriber = node.create_subscriber::<Joy>("joy", None)
        .context("Failed to create joy subscriber")?;
    
    // Create publishers based on profile
    let publishers = Arc::new(Publishers::from_profile(&node, &profile)
        .map_err(|e| JoyRouterError::PublisherError(format!("Failed to create publishers: {}", e)))?);

    pr_info!(logger, "Joy message router node started");
    
    // Log profile details
    if let Some(button) = profile.enable_button {
        pr_info!(logger, "Enable button: {}", button);
    }
    pr_info!(logger, "Axis mappings: {} configured", profile.axis_mappings.len());
    pr_info!(logger, "Button mappings: {} configured", profile.button_mappings.len());

    // Create a button tracker
    let button_tracker = Arc::new(Mutex::new(ButtonTracker::new()));
    let _enable_state = EnableStateTracker { _was_enabled: false };
    
    // Create command queue for decoupled processing
    let command_queue = Arc::new(CommandQueue::new());
    let queue_sender = command_queue.get_sender();
    
    // Store the last received joy axes
    let last_joy_axes = Arc::new(Mutex::new(Vec::<f32>::new()));
    
    // Create a selector for handling callbacks
    let mut selector = ctx.create_selector()
        .context("Failed to create selector")?;

    let button_tracker_sub = Arc::clone(&button_tracker);
    let last_joy_axes_sub = Arc::clone(&last_joy_axes);
    
    selector.add_subscriber(
        joy_subscriber,
        Box::new(move |msg| {
            pr_debug!(logger, "Received Joy message");
            
            // Update button tracker
            match button_tracker_sub.lock() {
                Ok(mut tracker) => tracker.update(msg.buttons.as_slice()),
                Err(e) => {
                    pr_error!(logger, "Failed to lock button tracker: {}", e);
                    return;
                }
            }
            
            // Store the axes for the timer callback
            match last_joy_axes_sub.lock() {
                Ok(mut last_axes) => *last_axes = msg.axes.as_slice().to_vec(),
                Err(e) => {
                    pr_error!(logger, "Failed to lock joy axes: {}", e);
                    return;
                }
            }
        }),
    );

    let button_tracker_timer = Arc::clone(&button_tracker);
    let last_joy_axes_timer = Arc::clone(&last_joy_axes);
    let queue_sender_timer = queue_sender.clone();
    let profile_clone = profile.clone();
    let logger_timer = Logger::new("joy_msg_router_timer");
    
    selector.add_wall_timer(
        "create_command",
        profile.timer_config.continuous_interval(),
        Box::new(move || {
            // Get the last joy axes
            let axes = match last_joy_axes_timer.lock() {
                Ok(axes_guard) => axes_guard.clone(),
                Err(e) => {
                    pr_error!(logger_timer, "Failed to lock joy axes: {}", e);
                    return;
                }
            };
            
            // If no joy message received yet, skip
            if axes.is_empty() {
                return;
            }
            
            // Check if output is enabled
            let tracker = match button_tracker_timer.lock() {
                Ok(t) => t,
                Err(e) => {
                    pr_error!(logger_timer, "Failed to lock button tracker: {}", e);
                    return;
                }
            };
            if !is_enabled(&profile_clone, &tracker) {
                pr_debug!(logger_timer, "Output disabled (enable button not pressed)");
                return;
            }
            
            // Process button actions first
            let mut button_twist_override = None;
            let button_count = tracker.button_count();
            for button_mapping in &profile_clone.button_mappings {
                if button_mapping.button < button_count {
                    let button_pressed = tracker.is_pressed(button_mapping.button);
                    let just_pressed = tracker.just_pressed(button_mapping.button);
                    let just_released = tracker.just_released(button_mapping.button);
                    
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
                                        
                                        pr_info!(logger_timer, "Button {} pressed - publishing one-shot twist", button_mapping.button);
                                        
                                        if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                            command: Command::PublishTwist(action_twist),
                                            priority: Priority::Normal,
                                        }) {
                                            pr_warn!(logger_timer, "Failed to enqueue one-shot Twist command: {:?}", e);
                                        }
                                    }
                                }
                            } else if button_pressed {
                                // Continuous action while button is held
                                if let Some(mut action_twist) = Twist::new() {
                                    action_twist.linear.x = *linear_x;
                                    action_twist.linear.y = *linear_y;
                                    action_twist.linear.z = *linear_z;
                                    action_twist.angular.x = *angular_x;
                                    action_twist.angular.y = *angular_y;
                                    action_twist.angular.z = *angular_z;
                                    
                                    button_twist_override = Some(action_twist);
                                    
                                    if just_pressed {
                                        pr_info!(logger_timer, "Button {} pressed - publishing continuous twist", button_mapping.button);
                                    }
                                }
                            }
                        }
                        ActionType::PublishTwistStamped { linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, frame_id, once } => {
                            if *once {
                                // One-shot action on button press
                                if just_pressed {
                                    if let Some(mut twist_stamped) = geometry_msgs::msg::TwistStamped::new() {
                                        // Create frame_id RosString
                                        if let Some(frame_id_ros) = safe_drive::msg::RosString::<0>::new(frame_id) {
                                            twist_stamped.header.frame_id = frame_id_ros;
                                        }
                                        twist_stamped.twist.linear.x = *linear_x;
                                        twist_stamped.twist.linear.y = *linear_y;
                                        twist_stamped.twist.linear.z = *linear_z;
                                        twist_stamped.twist.angular.x = *angular_x;
                                        twist_stamped.twist.angular.y = *angular_y;
                                        twist_stamped.twist.angular.z = *angular_z;
                                        
                                        pr_info!(logger_timer, "Button {} pressed - publishing one-shot twist stamped", button_mapping.button);
                                        
                                        if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                            command: Command::PublishTwistStamped(twist_stamped),
                                            priority: Priority::Normal,
                                        }) {
                                            pr_warn!(logger_timer, "Failed to enqueue TwistStamped command: {:?}", e);
                                        }
                                    }
                                }
                            } else if button_pressed && just_pressed {
                                pr_info!(logger_timer, "Button {} - continuous TwistStamped not implemented yet", button_mapping.button);
                            }
                        }
                        ActionType::PublishBool { topic, value, once } => {
                            if *once {
                                if just_pressed {
                                    if let Some(mut bool_msg) = std_msgs::msg::Bool::new() {
                                        bool_msg.data = *value;
                                        
                                        pr_info!(logger_timer, "Button {} pressed - publishing Bool {} to {}", button_mapping.button, value, topic);
                                        
                                        if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                            command: Command::PublishBool { topic: topic.clone(), value: bool_msg },
                                            priority: Priority::Normal,
                                        }) {
                                            pr_warn!(logger_timer, "Failed to enqueue Bool command: {:?}", e);
                                        }
                                    }
                                }
                            } else if button_pressed {
                                if let Some(mut bool_msg) = std_msgs::msg::Bool::new() {
                                    bool_msg.data = *value;
                                    
                                    if just_pressed {
                                        pr_info!(logger_timer, "Button {} pressed - continuous Bool {} to {}", button_mapping.button, value, topic);
                                    }
                                    
                                    if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                        command: Command::PublishBool { topic: topic.clone(), value: bool_msg },
                                        priority: Priority::Normal,
                                    }) {
                                        pr_debug!(logger_timer, "Failed to enqueue Bool command: {:?}", e);
                                    }
                                }
                            }
                        }
                        ActionType::PublishInt32 { topic, value, once } => {
                            if *once {
                                if just_pressed {
                                    if let Some(mut int_msg) = std_msgs::msg::Int32::new() {
                                        int_msg.data = *value;
                                        
                                        pr_info!(logger_timer, "Button {} pressed - publishing Int32 {} to {}", button_mapping.button, value, topic);
                                        
                                        if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                            command: Command::PublishInt32 { topic: topic.clone(), value: int_msg },
                                            priority: Priority::Normal,
                                        }) {
                                            pr_info!(logger_timer, "Failed to enqueue Int32 command: {:?}", e);
                                        }
                                    }
                                }
                            } else if button_pressed {
                                if let Some(mut int_msg) = std_msgs::msg::Int32::new() {
                                    int_msg.data = *value;
                                    
                                    if just_pressed {
                                        pr_info!(logger_timer, "Button {} pressed - continuous Int32 {} to {}", button_mapping.button, value, topic);
                                    }
                                    
                                    if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                        command: Command::PublishInt32 { topic: topic.clone(), value: int_msg },
                                        priority: Priority::Normal,
                                    }) {
                                        pr_debug!(logger_timer, "Failed to enqueue Int32 command: {:?}", e);
                                    }
                                }
                            }
                        }
                        ActionType::PublishFloat64 { topic, value, once } => {
                            if *once {
                                if just_pressed {
                                    if let Some(mut float_msg) = std_msgs::msg::Float64::new() {
                                        float_msg.data = *value;
                                        
                                        pr_info!(logger_timer, "Button {} pressed - publishing Float64 {} to {}", button_mapping.button, value, topic);
                                        
                                        if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                            command: Command::PublishFloat64 { topic: topic.clone(), value: float_msg },
                                            priority: Priority::Normal,
                                        }) {
                                            pr_info!(logger_timer, "Failed to enqueue Float64 command: {:?}", e);
                                        }
                                    }
                                }
                            } else if button_pressed {
                                if let Some(mut float_msg) = std_msgs::msg::Float64::new() {
                                    float_msg.data = *value;
                                    
                                    if just_pressed {
                                        pr_info!(logger_timer, "Button {} pressed - continuous Float64 {} to {}", button_mapping.button, value, topic);
                                    }
                                    
                                    if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                        command: Command::PublishFloat64 { topic: topic.clone(), value: float_msg },
                                        priority: Priority::Normal,
                                    }) {
                                        pr_debug!(logger_timer, "Failed to enqueue Float64 command: {:?}", e);
                                    }
                                }
                            }
                        }
                        ActionType::PublishString { topic, value, once } => {
                            if *once {
                                if just_pressed {
                                    if let Some(mut str_msg) = std_msgs::msg::String::new() {
                                        // Create data RosString
                                        if let Some(data_ros) = safe_drive::msg::RosString::<0>::new(value) {
                                            str_msg.data = data_ros;
                                        }
                                        
                                        pr_info!(logger_timer, "Button {} pressed - publishing String '{}' to {}", button_mapping.button, value, topic);
                                        
                                        if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                            command: Command::PublishString { topic: topic.clone(), value: str_msg },
                                            priority: Priority::Normal,
                                        }) {
                                            pr_info!(logger_timer, "Failed to enqueue String command: {:?}", e);
                                        }
                                    }
                                }
                            } else if button_pressed {
                                if let Some(mut str_msg) = std_msgs::msg::String::new() {
                                    // Create data RosString
                                    if let Some(data_ros) = safe_drive::msg::RosString::<0>::new(value) {
                                        str_msg.data = data_ros;
                                    }
                                    
                                    if just_pressed {
                                        pr_info!(logger_timer, "Button {} pressed - continuous String '{}' to {}", button_mapping.button, value, topic);
                                    }
                                    
                                    if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                        command: Command::PublishString { topic: topic.clone(), value: str_msg },
                                        priority: Priority::Normal,
                                    }) {
                                        pr_debug!(logger_timer, "Failed to enqueue String command: {:?}", e);
                                    }
                                }
                            }
                        }
                        ActionType::CallService { service_name, service_type, once } => {
                            if *once {
                                // One-shot service call on button press
                                if just_pressed {
                                    pr_info!(logger_timer, "Button {} pressed - calling service once: {}", 
                                            button_mapping.button, service_name);
                                    
                                    if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                        command: Command::CallService {
                                            service_name: service_name.clone(),
                                            service_type: service_type.clone(),
                                        },
                                        priority: Priority::High,
                                    }) {
                                        pr_info!(logger_timer, "Failed to enqueue service call: {:?}", e);
                                    }
                                }
                            } else if button_pressed {
                                // Continuous service calls while button is held
                                if just_pressed {
                                    pr_info!(logger_timer, "Button {} pressed - starting continuous service calls: {}", 
                                            button_mapping.button, service_name);
                                }
                                
                                if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                    command: Command::CallService {
                                        service_name: service_name.clone(),
                                        service_type: service_type.clone(),
                                    },
                                    priority: Priority::Normal,
                                }) {
                                    pr_debug!(logger_timer, "Failed to enqueue continuous service call: {:?}", e);
                                }
                                
                                if just_released {
                                    pr_info!(logger_timer, "Button {} released - stopping continuous service calls: {}", 
                                            button_mapping.button, service_name);
                                }
                            }
                        }
                        ActionType::NoAction => {
                            if just_pressed {
                                pr_info!(logger_timer, "Button {} pressed - stop action", button_mapping.button);
                                
                                // Send stop command
                                if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                                    command: Command::Stop,
                                    priority: Priority::High,
                                }) {
                                    pr_info!(logger_timer, "Failed to enqueue stop command: {:?}", e);
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
                match axes_to_twist(&axes, &profile_clone) {
                    Ok(twist) => twist,
                    Err(e) => {
                        pr_debug!(logger_timer, "Failed to convert axes to twist: {}", e);
                        return;
                    }
                }
            };
            
            // Check if it's a non-zero twist before publishing
            if twist_to_publish.linear.x != 0.0 || twist_to_publish.linear.y != 0.0 ||
               twist_to_publish.linear.z != 0.0 || twist_to_publish.angular.x != 0.0 ||
               twist_to_publish.angular.y != 0.0 || twist_to_publish.angular.z != 0.0 {
                pr_debug!(
                    logger_timer,
                    "Publishing Twist: linear=({:.2}, {:.2}, {:.2}), angular=({:.2}, {:.2}, {:.2})",
                    twist_to_publish.linear.x,
                    twist_to_publish.linear.y,
                    twist_to_publish.linear.z,
                    twist_to_publish.angular.x,
                    twist_to_publish.angular.y,
                    twist_to_publish.angular.z
                );
                
                // Enqueue the Twist command
                if let Err(e) = queue_sender_timer.send(command_queue::PrioritizedCommand {
                    command: Command::PublishTwist(twist_to_publish),
                    priority: Priority::Normal,
                }) {
                    pr_info!(logger_timer, "Failed to enqueue Twist command: {:?}", e);
                }
            }
        }),
    );

    
    // Create a separate logger for the command processing loop
    let process_logger = Logger::new("joy_msg_router_cmd_processor");
    let publishers_for_processing = Arc::clone(&publishers);
    
    // Spin the selector and process commands
    loop {
        // Wait for events with a timeout to allow command processing
        selector.wait_timeout(std::time::Duration::from_millis(10))
            .context("Selector wait failed")?;
        
        // Process pending commands
        command_queue.process_pending(|cmd| {
            let command_str = format!("{:?}", cmd.command);
            let result = match cmd.command {
                Command::PublishTwist(twist) => {
                    publishers_for_processing.twist_publisher.send(&twist)
                        .map_err(|e| JoyRouterError::PublisherError(
                            format!("Failed to publish Twist message: {}", e)
                        ))
                }
                Command::PublishTwistStamped(twist_stamped) => {
                    if let Some(ref publisher) = publishers_for_processing.twist_stamped_publisher {
                        publisher.send(&twist_stamped)
                            .map_err(|e| JoyRouterError::PublisherError(
                                format!("Failed to publish TwistStamped message: {}", e)
                            ))
                    } else {
                        Err(JoyRouterError::PublisherError(
                            "TwistStamped publisher not configured".to_string()
                        ))
                    }
                }
                Command::PublishBool { topic, value } => {
                    if let Some(publisher) = publishers_for_processing.bool_publishers.get(&topic) {
                        publisher.send(&value)
                            .map_err(|e| JoyRouterError::PublisherError(
                                format!("Failed to publish Bool message to {}: {}", topic, e)
                            ))
                    } else {
                        Err(JoyRouterError::PublisherError(
                            format!("Bool publisher for topic '{}' not configured", topic)
                        ))
                    }
                }
                Command::PublishInt32 { topic, value } => {
                    if let Some(publisher) = publishers_for_processing.int32_publishers.get(&topic) {
                        publisher.send(&value)
                            .map_err(|e| JoyRouterError::PublisherError(
                                format!("Failed to publish Int32 message to {}: {}", topic, e)
                            ))
                    } else {
                        Err(JoyRouterError::PublisherError(
                            format!("Int32 publisher for topic '{}' not configured", topic)
                        ))
                    }
                }
                Command::PublishFloat64 { topic, value } => {
                    if let Some(publisher) = publishers_for_processing.float64_publishers.get(&topic) {
                        publisher.send(&value)
                            .map_err(|e| JoyRouterError::PublisherError(
                                format!("Failed to publish Float64 message to {}: {}", topic, e)
                            ))
                    } else {
                        Err(JoyRouterError::PublisherError(
                            format!("Float64 publisher for topic '{}' not configured", topic)
                        ))
                    }
                }
                Command::PublishString { topic, value } => {
                    if let Some(publisher) = publishers_for_processing.string_publishers.get(&topic) {
                        publisher.send(&value)
                            .map_err(|e| JoyRouterError::PublisherError(
                                format!("Failed to publish String message to {}: {}", topic, e)
                            ))
                    } else {
                        Err(JoyRouterError::PublisherError(
                            format!("String publisher for topic '{}' not configured", topic)
                        ))
                    }
                }
                Command::CallService { service_name, service_type } => {
                    // TODO: Implement actual service client when safe_drive supports it
                    log_info!(&process_logger, "service_client", "call", 
                        &format!("Would call service: {} (type: {})", service_name, service_type));
                    Ok(())
                }
                Command::Stop => {
                    // Send zero twist to cmd_vel
                    if let Some(zero_twist) = Twist::new() {
                        publishers_for_processing.twist_publisher.send(&zero_twist)
                            .map_err(|e| JoyRouterError::PublisherError(
                                format!("Failed to publish stop command: {}", e)
                            ))
                    } else {
                        Err(JoyRouterError::MessageError(
                            "Failed to create zero Twist message".to_string()
                        ))
                    }
                }
            };
            
            // Log command result
            log_command_result(&process_logger, &command_str, result);
            Ok(())
        })
        .map_err(|e| JoyRouterError::CommandError(
            format!("Failed to process command queue: {}", e)
        ))?;
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
        let _joy = create_test_joy(vec![], vec![0, 1, 0]);
        let profile = Profile::new("test".to_string()); // No enable button
        let tracker = Arc::new(Mutex::new(ButtonTracker::new()));
        {
            let mut tracker_guard = tracker.lock().unwrap();
            tracker_guard.update(&[0, 1, 0]);
        }
        let tracker_guard = tracker.lock().unwrap();

        assert!(is_enabled(&profile, &tracker_guard));
    }

    #[test]
    fn test_is_enabled_with_button() {
        let _joy = create_test_joy(vec![], vec![0, 1, 0]);
        let mut profile = Profile::new("test".to_string());
        let tracker = Arc::new(Mutex::new(ButtonTracker::new()));
        {
            let mut tracker_guard = tracker.lock().unwrap();
            tracker_guard.update(&[0, 1, 0]);
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
        let tracker = Arc::new(Mutex::new(ButtonTracker::new()));
        {
            let mut tracker_guard = tracker.lock().unwrap();
            tracker_guard.update(&[0, 0, 1, 0]);
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
        let _joy = create_test_joy(vec![], vec![1, 0, 1, 0]);
        let mut profile = Profile::new("test".to_string());
        let tracker = Arc::new(Mutex::new(ButtonTracker::new()));
        {
            let mut tracker_guard = tracker.lock().unwrap();
            tracker_guard.update(&[1, 0, 1, 0]);
        }

        // Test when both single and multiple buttons are configured
        profile.enable_button = Some(1); // Not pressed
        profile.enable_buttons = Some(vec![2, 3]); // Button 2 is pressed
        
        // Should be enabled because button 2 is pressed (OR logic between single and multiple)
        {
            let tracker_guard = tracker.lock().unwrap();
            assert!(is_enabled(&profile, &tracker_guard));
        }
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
