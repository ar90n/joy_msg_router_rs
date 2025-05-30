use anyhow::{Result, anyhow};
use serde::{Deserialize, Serialize};

/// Represents a configuration profile for joy message routing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Profile {
    /// Name of the profile
    pub name: String,

    /// Optional button index that must be pressed to enable output
    pub enable_button: Option<usize>,

    /// Input mappings (both axes and buttons)
    pub input_mappings: Vec<InputMapping>,
}

/// Type of input source
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InputSource {
    /// Axis input (continuous value)
    Axis(usize),
    /// Button input (discrete on/off)
    Button(usize),
}

/// Represents a mapping from an input to an action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputMapping {
    /// Input source (axis or button)
    pub source: InputSource,
    
    /// Action to perform
    pub action: ActionType,
    
    /// Scale factor for axis values (ignored for buttons)
    #[serde(default = "default_scale")]
    pub scale: f64,
    
    /// Offset to add after scaling (ignored for buttons)
    #[serde(default)]
    pub offset: f64,
    
    /// Deadzone threshold for axes (ignored for buttons)
    #[serde(default = "default_deadzone")]
    pub deadzone: f64,
}

fn default_scale() -> f64 {
    1.0
}

fn default_deadzone() -> f64 {
    0.1
}


/// Enum representing different action types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ActionType {
    /// Publish to a Twist field
    PublishTwistField {
        /// Which field to publish to ("linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z")
        field: String,
    },


    /// Publish a Bool message
    PublishBool {
        topic: String,
        value: bool,
        /// Whether to publish only once (true) or continuously while active (false)
        #[serde(default = "default_true")]
        once: bool,
    },

    /// Publish an Int32 message
    PublishInt32 {
        topic: String,
        value: i32,
        /// Whether to publish only once (true) or continuously while active (false)
        #[serde(default = "default_true")]
        once: bool,
    },

    /// Publish a Float64 message
    PublishFloat64 {
        topic: String,
        value: f64,
        /// Whether to publish only once (true) or continuously while active (false)
        #[serde(default = "default_true")]
        once: bool,
    },

    /// Publish a String message
    PublishString {
        topic: String,
        value: String,
        /// Whether to publish only once (true) or continuously while active (false)
        #[serde(default = "default_true")]
        once: bool,
    },

    /// Call a service
    CallService {
        service_name: String,
        service_type: String,
    },
}

fn default_true() -> bool {
    true
}

impl Profile {
    /// Creates a new empty profile
    pub fn new(name: String) -> Self {
        Self {
            name,
            enable_button: None,
            input_mappings: Vec::new(),
        }
    }

    /// Validates the profile configuration
    pub fn validate(&self) -> Result<()> {
        // Check for duplicate input sources
        let mut used_sources = std::collections::HashSet::new();
        for mapping in &self.input_mappings {
            let source_key = match mapping.source {
                InputSource::Axis(idx) => format!("axis_{}", idx),
                InputSource::Button(idx) => format!("button_{}", idx),
            };
            
            if !used_sources.insert(source_key.clone()) {
                return Err(anyhow!(format!(
                    "Duplicate mapping for {}",
                    source_key
                )));
            }
        }

        // Validate axis-specific settings
        for mapping in &self.input_mappings {
            if let InputSource::Axis(_) = mapping.source {
                if mapping.deadzone < 0.0 {
                    return Err(anyhow!(format!(
                        "Negative deadzone for {:?}",
                        mapping.source
                    )));
                }
                if mapping.scale == 0.0 {
                    return Err(anyhow!(format!(
                        "Zero scale for {:?}",
                        mapping.source
                    )));
                }
            }
        }

        Ok(())
    }
}

impl InputMapping {
    pub fn process_value(&self, value: f64) -> f64 {
        match self.source {
            InputSource::Axis(_) => {
                if value.abs() < self.deadzone {
                    0.0
                } else {
                    value * self.scale + self.offset
                }
            }
            InputSource::Button(_) => {
                if value > 0.5 { 1.0 } else { 0.0 }
            }
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_axis_input_processing() {
        let mapping = InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::PublishTwistField {
                field: "linear_x".to_string(),
            },
            scale: 2.0,
            offset: 0.5,
            deadzone: 0.1,
        };

        assert_eq!(mapping.process_value(0.05), 0.0);
        assert_eq!(mapping.process_value(0.5), 1.5);
        assert_eq!(mapping.process_value(-0.5), -0.5);
    }

    #[test]
    fn test_button_input_processing() {
        let mapping = InputMapping {
            source: InputSource::Button(0),
            action: ActionType::PublishBool {
                topic: "test".to_string(),
                value: true,
                once: true,
            },
            scale: 2.0,
            offset: 0.5,
            deadzone: 0.1,
        };

        assert_eq!(mapping.process_value(0.0), 0.0);
        assert_eq!(mapping.process_value(1.0), 1.0);
        assert_eq!(mapping.process_value(0.3), 0.0);
        assert_eq!(mapping.process_value(0.7), 1.0);
    }

    #[test]
    fn test_profile_validation() {
        let mut profile = Profile::new("test".to_string());

        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::PublishTwistField {
                field: "linear_x".to_string(),
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.1,
        });
        
        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::PublishTwistField {
                field: "angular_z".to_string(),
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.1,
        });

        assert!(profile.validate().is_err());
    }

    #[test]
    fn test_profile_validation_negative_deadzone() {
        let mut profile = Profile::new("test".to_string());

        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::PublishTwistField {
                field: "linear_x".to_string(),
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: -0.1,
        });

        assert!(profile.validate().is_err());
    }

    #[test]
    fn test_profile_validation_zero_scale() {
        let mut profile = Profile::new("test".to_string());

        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::PublishTwistField {
                field: "linear_x".to_string(),
            },
            scale: 0.0,
            offset: 0.0,
            deadzone: 0.1,
        });

        assert!(profile.validate().is_err());
    }

    #[test]
    fn test_profile_validation_duplicate_buttons() {
        let mut profile = Profile::new("test".to_string());

        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(0),
            action: ActionType::PublishBool {
                topic: "test".to_string(),
                value: true,
                once: true,
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.0,
        });

        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(0),
            action: ActionType::CallService {
                service_name: "test".to_string(),
                service_type: "std_srvs/Trigger".to_string(),
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.0,
        });

        assert!(profile.validate().is_err());
    }
}