use serde::{Deserialize, Serialize};
use std::time::Duration;
use crate::error::{JoyRouterError, JoyRouterResult};

/// Represents a configuration profile for joy message routing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Profile {
    /// Name of the profile
    pub name: String,
    
    /// Optional button index that must be pressed to enable output
    pub enable_button: Option<usize>,
    
    /// Optional list of button indices (any must be pressed to enable)
    pub enable_buttons: Option<Vec<usize>>,
    
    /// Axis to output field mappings
    pub axis_mappings: Vec<AxisMapping>,
    
    /// Button to action mappings
    pub button_mappings: Vec<ButtonMapping>,
    
    /// Timer configuration for periodic actions
    #[serde(default)]
    pub timer_config: TimerConfig,
    
    /// Macro definitions available in this profile
    #[serde(default)]
    pub macros: std::collections::HashMap<String, MacroDefinition>,
    
    
}

/// Timer configuration for periodic command execution
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimerConfig {
    /// Interval for continuous action timers (in milliseconds)
    #[serde(default = "TimerConfig::default_continuous_interval")]
    pub continuous_interval_ms: u64,
    
    /// Interval for safety stop timer (in milliseconds)
    #[serde(default = "TimerConfig::default_safety_interval")]
    pub safety_stop_interval_ms: u64,
    
    /// Enable safety stop timer
    #[serde(default = "TimerConfig::default_enable_safety")]
    pub enable_safety_stop: bool,
}

impl Default for TimerConfig {
    fn default() -> Self {
        Self {
            continuous_interval_ms: Self::default_continuous_interval(),
            safety_stop_interval_ms: Self::default_safety_interval(),
            enable_safety_stop: Self::default_enable_safety(),
        }
    }
}

impl TimerConfig {
    fn default_continuous_interval() -> u64 { 50 }  // 50ms = 20Hz
    fn default_safety_interval() -> u64 { 500 }     // 500ms
    fn default_enable_safety() -> bool { false }
    
    pub fn continuous_interval(&self) -> Duration {
        Duration::from_millis(self.continuous_interval_ms)
    }
    
    pub fn safety_interval(&self) -> Duration {
        Duration::from_millis(self.safety_stop_interval_ms)
    }
}

/// Represents a mapping from a joy axis to an output field
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AxisMapping {
    /// Index of the joy axis
    pub joy_axis: usize,
    
    /// Output field (e.g., "linear.x", "angular.z")
    pub output_field: OutputField,
    
    /// Scale factor to apply to the axis value
    pub scale: f64,
    
    /// Offset to add after scaling
    pub offset: f64,
    
    /// Deadzone threshold (values below this are treated as 0)
    pub deadzone: f64,
}

/// Represents a mapping from a button to an action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ButtonMapping {
    /// Index of the joy button
    pub button: usize,
    
    /// Action to perform when button is pressed
    pub action: ActionType,
}

/// Enum representing different output fields for Twist messages
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputField {
    LinearX,
    LinearY,
    LinearZ,
    AngularX,
    AngularY,
    AngularZ,
}

impl OutputField {
    /// Parse from string representation
    pub fn from_str(s: &str) -> JoyRouterResult<Self> {
        match s {
            "linear_x" => Ok(OutputField::LinearX),
            "linear_y" => Ok(OutputField::LinearY),
            "linear_z" => Ok(OutputField::LinearZ),
            "angular_x" => Ok(OutputField::AngularX),
            "angular_y" => Ok(OutputField::AngularY),
            "angular_z" => Ok(OutputField::AngularZ),
            _ => Err(JoyRouterError::ConfigError(
                format!("Invalid output field: '{}'. Expected one of: linear_x, linear_y, linear_z, angular_x, angular_y, angular_z", s)
            )),
        }
    }
}

/// Enum representing different action types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ActionType {
    /// Publish a Twist message
    PublishTwist {
        linear_x: f64,
        linear_y: f64,
        linear_z: f64,
        angular_x: f64,
        angular_y: f64,
        angular_z: f64,
        /// Whether to publish only once (true) or continuously while pressed (false)
        #[serde(default)]
        once: bool,
    },
    
    /// Publish a TwistStamped message
    PublishTwistStamped {
        linear_x: f64,
        linear_y: f64,
        linear_z: f64,
        angular_x: f64,
        angular_y: f64,
        angular_z: f64,
        frame_id: String,
        /// Whether to publish only once (true) or continuously while pressed (false)
        #[serde(default)]
        once: bool,
    },
    
    /// Publish a Bool message
    PublishBool {
        topic: String,
        value: bool,
        /// Whether to publish only once (true) or continuously while pressed (false)
        #[serde(default = "default_true")]
        once: bool,
    },
    
    /// Publish an Int32 message
    PublishInt32 {
        topic: String,
        value: i32,
        /// Whether to publish only once (true) or continuously while pressed (false)
        #[serde(default = "default_true")]
        once: bool,
    },
    
    /// Publish a Float64 message
    PublishFloat64 {
        topic: String,
        value: f64,
        /// Whether to publish only once (true) or continuously while pressed (false)
        #[serde(default = "default_true")]
        once: bool,
    },
    
    /// Publish a String message
    PublishString {
        topic: String,
        value: String,
        /// Whether to publish only once (true) or continuously while pressed (false)
        #[serde(default = "default_true")]
        once: bool,
    },
    
    /// Call a service
    CallService {
        service_name: String,
        service_type: String,
        /// Whether to call only once (true) or continuously while pressed (false)
        #[serde(default)]
        once: bool,
    },
    
    /// Execute a sequence of actions
    ActionSequence {
        /// List of actions to execute in sequence
        actions: Vec<SequenceStep>,
        /// Whether to execute only once (true) or continuously while pressed (false)
        #[serde(default)]
        once: bool,
        /// Whether to repeat the sequence (only valid if once=false)
        #[serde(default)]
        repeat: bool,
    },
    
    /// Execute a predefined macro
    ExecuteMacro {
        /// Name of the macro to execute
        macro_name: String,
        /// Parameters to pass to the macro
        #[serde(default)]
        parameters: std::collections::HashMap<String, MacroParameter>,
        /// Whether to execute only once (true) or continuously while pressed (false)
        #[serde(default)]
        once: bool,
    },
    
    
    
    /// No action (stop)
    NoAction,
}

fn default_true() -> bool {
    true
}

/// A step in an action sequence
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SequenceStep {
    /// The action to execute
    pub action: ActionType,
    /// Delay before executing this step (in milliseconds)
    #[serde(default)]
    pub delay_ms: u64,
    /// Duration to hold this action (in milliseconds, 0 means instantaneous)
    #[serde(default)]
    pub duration_ms: u64,
}

/// Parameter value for macros
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum MacroParameter {
    String(String),
    Number(f64),
    Boolean(bool),
    Array(Vec<MacroParameter>),
}

/// A macro definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MacroDefinition {
    /// Name of the macro
    pub name: String,
    /// Description of what the macro does
    #[serde(default)]
    pub description: String,
    /// Parameter definitions
    #[serde(default)]
    pub parameters: std::collections::HashMap<String, MacroParameterDef>,
    /// Sequence of actions to execute
    pub actions: Vec<SequenceStep>,
}

/// Definition of a macro parameter
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MacroParameterDef {
    /// Parameter type
    #[serde(rename = "type")]
    pub param_type: String,
    /// Default value
    #[serde(default)]
    pub default: Option<MacroParameter>,
    /// Description of the parameter
    #[serde(default)]
    pub description: String,
}



impl Profile {
    /// Creates a new empty profile
    pub fn new(name: String) -> Self {
        Self {
            name,
            enable_button: None,
            enable_buttons: None,
            axis_mappings: Vec::new(),
            button_mappings: Vec::new(),
            timer_config: TimerConfig::default(),
            macros: std::collections::HashMap::new(),
        }
    }
    
    /// Validates the profile configuration
    pub fn validate(&self) -> JoyRouterResult<()> {
        // Check for duplicate axis mappings
        let mut used_axes = std::collections::HashSet::new();
        for mapping in &self.axis_mappings {
            if !used_axes.insert(mapping.joy_axis) {
                return Err(JoyRouterError::ConfigError(
                    format!("Duplicate mapping for axis {}", mapping.joy_axis)
                ));
            }
        }
        
        // Check for duplicate button mappings
        let mut used_buttons = std::collections::HashSet::new();
        for mapping in &self.button_mappings {
            if !used_buttons.insert(mapping.button) {
                return Err(JoyRouterError::ConfigError(
                    format!("Duplicate mapping for button {}", mapping.button)
                ));
            }
        }
        
        // Validate scale and deadzone values
        for mapping in &self.axis_mappings {
            if mapping.deadzone < 0.0 {
                return Err(JoyRouterError::ConfigError(
                    format!("Negative deadzone for axis {}", mapping.joy_axis)
                ));
            }
            if mapping.scale == 0.0 {
                return Err(JoyRouterError::ConfigError(
                    format!("Zero scale for axis {}", mapping.joy_axis)
                ));
            }
        }
        
        Ok(())
    }
}

impl AxisMapping {
    /// Applies the mapping to an input value
    pub fn apply(&self, value: f64) -> f64 {
        if value.abs() < self.deadzone {
            0.0
        } else {
            value * self.scale + self.offset
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_axis_mapping_apply() {
        let mapping = AxisMapping {
            joy_axis: 0,
            output_field: OutputField::LinearX,
            scale: 2.0,
            offset: 0.5,
            deadzone: 0.1,
        };
        
        assert_eq!(mapping.apply(0.05), 0.0); // Within deadzone
        assert_eq!(mapping.apply(0.5), 1.5); // 0.5 * 2.0 + 0.5
        assert_eq!(mapping.apply(-0.5), -0.5); // -0.5 * 2.0 + 0.5
    }
    
    #[test]
    fn test_profile_validation() {
        let mut profile = Profile::new("test".to_string());
        
        // Add duplicate axis mapping
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 0,
            output_field: OutputField::LinearX,
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.1,
        });
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 0, // Duplicate
            output_field: OutputField::AngularZ,
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.1,
        });
        
        assert!(profile.validate().is_err());
    }
    
    #[test]
    fn test_axis_mapping_apply_edge_cases() {
        let mapping = AxisMapping {
            joy_axis: 0,
            output_field: OutputField::LinearX,
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.1,
        };
        
        // Test exact deadzone boundary - should pass through since we use < not <=
        assert_eq!(mapping.apply(0.1), 0.1); // Exactly at deadzone threshold (not within)
        assert_eq!(mapping.apply(-0.1), -0.1); // Negative deadzone boundary
        
        // Test just inside deadzone
        assert_eq!(mapping.apply(0.09), 0.0); // Should be zero inside deadzone
        assert_eq!(mapping.apply(-0.09), 0.0); // Should be zero inside negative deadzone
        
        // Test just outside deadzone
        assert!((mapping.apply(0.11) - 0.11).abs() < f64::EPSILON);
        assert!((mapping.apply(-0.11) - (-0.11)).abs() < f64::EPSILON);
    }
    
    #[test]
    fn test_axis_mapping_with_offset_and_deadzone() {
        let mapping = AxisMapping {
            joy_axis: 0,
            output_field: OutputField::LinearX,
            scale: 2.0,
            offset: 0.5,
            deadzone: 0.2,
        };
        
        // Within deadzone should return 0, not offset
        assert_eq!(mapping.apply(0.1), 0.0);
        assert_eq!(mapping.apply(-0.1), 0.0);
        
        // Outside deadzone should apply scale and offset
        assert_eq!(mapping.apply(0.5), 1.5); // 0.5 * 2.0 + 0.5
        assert_eq!(mapping.apply(-0.5), -0.5); // -0.5 * 2.0 + 0.5
    }
    
    #[test]
    fn test_profile_validation_negative_deadzone() {
        let mut profile = Profile::new("test".to_string());
        
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 0,
            output_field: OutputField::LinearX,
            scale: 1.0,
            offset: 0.0,
            deadzone: -0.1, // Invalid negative deadzone
        });
        
        assert!(profile.validate().is_err());
    }
    
    #[test]
    fn test_profile_validation_zero_scale() {
        let mut profile = Profile::new("test".to_string());
        
        profile.axis_mappings.push(AxisMapping {
            joy_axis: 0,
            output_field: OutputField::LinearX,
            scale: 0.0, // Invalid zero scale
            offset: 0.0,
            deadzone: 0.1,
        });
        
        assert!(profile.validate().is_err());
    }
    
    #[test]
    fn test_profile_validation_duplicate_buttons() {
        let mut profile = Profile::new("test".to_string());
        
        // Add duplicate button mappings
        profile.button_mappings.push(ButtonMapping {
            button: 0,
            action: ActionType::PublishTwist {
                linear_x: 0.0,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_x: 0.0,
                angular_y: 0.0,
                angular_z: 0.0,
                once: false,
            },
        });
        
        profile.button_mappings.push(ButtonMapping {
            button: 0, // Duplicate
            action: ActionType::CallService {
                service_name: "test".to_string(),
                service_type: "std_srvs/Trigger".to_string(),
                once: true,
            },
        });
        
        assert!(profile.validate().is_err());
    }
}