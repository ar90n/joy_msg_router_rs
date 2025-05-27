use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info, pr_debug,
};

// Import ROS message types
use sensor_msgs::msg::Joy;
use geometry_msgs::msg::Twist;

fn main() -> Result<(), DynError> {
    // Create a context.
    let ctx = Context::new()?;

    // Create a node.
    let node = ctx.create_node("joy_msg_router", None, Default::default())?;

    // Create a logger.
    let logger = Logger::new("joy_msg_router");

    // Create a subscriber for Joy messages
    let subscriber = node.create_subscriber::<Joy>("joy", None)?;

    pr_info!(logger, "Joy message router node started");
    pr_info!(logger, "Listening for Joy messages on /joy topic");

    // Create a selector for handling callbacks
    let mut selector = ctx.create_selector()?;
    selector.add_subscriber(
        subscriber, 
        Box::new(move |msg| {
            pr_debug!(logger, "Received Joy message");
            
            // Log axes values
            if !msg.axes.is_empty() {
                let axes_str: Vec<String> = msg.axes.iter()
                    .enumerate()
                    .map(|(i, &val)| format!("[{}]={:.3}", i, val))
                    .collect();
                pr_info!(logger, "Axes: {}", axes_str.join(", "));
            }
            
            // Log button values  
            if !msg.buttons.is_empty() {
                let buttons_str: Vec<String> = msg.buttons.iter()
                    .enumerate()
                    .filter(|(_, &val)| val != 0)
                    .map(|(i, &val)| format!("[{}]={}", i, val))
                    .collect();
                
                if !buttons_str.is_empty() {
                    pr_info!(logger, "Buttons pressed: {}", buttons_str.join(", "));
                }
            }
        }),
    );

    // Spin the selector
    loop {
        selector.wait()?;
    }
}
