use std::fmt;

/// Custom error types for joy_msg_router
#[derive(Debug)]
pub enum JoyRouterError {
    /// Configuration-related errors
    ConfigError(String),
    /// Publisher creation errors
    PublisherError(String),
    /// Command processing errors
    CommandError(String),
    /// Button state errors
    ButtonError(String),
    /// Timer-related errors
    TimerError(String),
    /// Message conversion errors
    MessageError(String),
    /// ROS communication errors
    RosError(String),
}

impl fmt::Display for JoyRouterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            JoyRouterError::ConfigError(msg) => write!(f, "Configuration error: {}", msg),
            JoyRouterError::PublisherError(msg) => write!(f, "Publisher error: {}", msg),
            JoyRouterError::CommandError(msg) => write!(f, "Command processing error: {}", msg),
            JoyRouterError::ButtonError(msg) => write!(f, "Button state error: {}", msg),
            JoyRouterError::TimerError(msg) => write!(f, "Timer error: {}", msg),
            JoyRouterError::MessageError(msg) => write!(f, "Message conversion error: {}", msg),
            JoyRouterError::RosError(msg) => write!(f, "ROS communication error: {}", msg),
        }
    }
}

impl std::error::Error for JoyRouterError {}


/// Convenience type alias for Results
pub type JoyRouterResult<T> = Result<T, JoyRouterError>;

/// Error context trait for adding context to errors
pub trait ErrorContext<T> {
    fn context(self, msg: &str) -> Result<T, JoyRouterError>;
}

impl<T, E: std::fmt::Display> ErrorContext<T> for Result<T, E> {
    fn context(self, msg: &str) -> Result<T, JoyRouterError> {
        self.map_err(|e| JoyRouterError::RosError(format!("{}: {}", msg, e)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = JoyRouterError::ConfigError("Invalid profile".to_string());
        assert_eq!(err.to_string(), "Configuration error: Invalid profile");
        
        let err = JoyRouterError::PublisherError("Failed to create publisher".to_string());
        assert_eq!(err.to_string(), "Publisher error: Failed to create publisher");
    }
    
    #[test]
    fn test_error_conversion() {
        let err = JoyRouterError::CommandError("Queue full".to_string());
        let _dyn_err: Box<dyn std::error::Error + Send + Sync> = Box::new(err);
    }
    
    #[test]
    fn test_error_context() {
        let result: Result<i32, std::io::Error> = Err(std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "file not found"
        ));
        
        let contextual = result.context("Failed to load config");
        assert!(contextual.is_err());
        match contextual {
            Err(JoyRouterError::RosError(msg)) => {
                assert!(msg.contains("Failed to load config"));
                assert!(msg.contains("file not found"));
            }
            _ => panic!("Expected RosError"),
        }
    }
}