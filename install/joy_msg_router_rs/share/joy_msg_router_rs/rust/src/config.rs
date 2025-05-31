use anyhow::{anyhow, Result};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Profile {
    pub name: String,
    pub enable_button: Option<usize>,
    pub input_mappings: Vec<InputMapping>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InputSource {
    Axis(usize),
    Button(usize),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputMapping {
    pub source: InputSource,
    pub action: ActionType,

    #[serde(default = "default_scale")]
    pub scale: f64,

    #[serde(default)]
    pub offset: f64,

    #[serde(default = "default_deadzone")]
    pub deadzone: f64,
}

fn default_scale() -> f64 {
    1.0
}

fn default_deadzone() -> f64 {
    0.1
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ActionType {
    /// Generic message publishing with runtime type specification
    /// Handles all message types including Twist
    Publish {
        topic: String,
        message_type: String, // e.g., "std_msgs/msg/Float64", "geometry_msgs/msg/Twist"
        /// Field to set in the message (e.g., "data" for std_msgs, "linear.x" for Twist)
        field: Option<String>,
        /// Whether to publish only once (true) or continuously while active (false)
        #[serde(default = "default_true")]
        once: bool,
    },

    CallService {
        service_name: String,
        service_type: String,
    },
}

fn default_true() -> bool {
    true
}

impl Profile {
    pub fn new(name: String) -> Self {
        Self {
            name,
            enable_button: None,
            input_mappings: Vec::new(),
        }
    }

    pub fn validate(&self) -> Result<()> {
        // Check for duplicate input sources
        let mut used_sources = std::collections::HashSet::new();
        for mapping in &self.input_mappings {
            let source_key = match mapping.source {
                InputSource::Axis(idx) => format!("axis_{}", idx),
                InputSource::Button(idx) => format!("button_{}", idx),
            };

            if !used_sources.insert(source_key.clone()) {
                return Err(anyhow!(format!("Duplicate mapping for {}", source_key)));
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
                    return Err(anyhow!(format!("Zero scale for {:?}", mapping.source)));
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
                if value > 0.5 {
                    1.0 * self.scale + self.offset
                } else {
                    0.0
                }
            }
        }
    }

    #[cfg(test)]
    pub fn process_input(&self, tracker: &crate::joy_msg_tracker::JoyMsgTracker) -> f64 {
        let raw_value = match self.source {
            InputSource::Axis(idx) => tracker.get_axis(idx).unwrap_or(0.0) as f64,
            InputSource::Button(idx) => {
                if tracker.is_pressed(idx) {
                    1.0
                } else {
                    0.0
                }
            }
        };
        self.process_value(raw_value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_axis_input_processing() {
        let mapping = InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
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
            action: ActionType::Publish {
                topic: "test".to_string(),
                message_type: "std_msgs/msg/Bool".to_string(),
                field: None,
                once: true,
            },
            scale: 2.0,
            offset: 0.5,
            deadzone: 0.1,
        };

        assert_eq!(mapping.process_value(0.0), 0.0);
        assert_eq!(mapping.process_value(1.0), 1.0 * 2.0 + 0.5); // scale and offset applied
        assert_eq!(mapping.process_value(0.3), 0.0);
        assert_eq!(mapping.process_value(0.7), 1.0 * 2.0 + 0.5); // scale and offset applied
    }

    #[test]
    fn test_profile_validation() {
        let mut profile = Profile::new("test".to_string());

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

        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("angular.z".to_string()),
                once: false,
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
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
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
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
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
            action: ActionType::Publish {
                topic: "test".to_string(),
                message_type: "std_msgs/msg/Bool".to_string(),
                field: None,
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
