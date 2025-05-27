use serde::{Deserialize, Serialize};

/// Represents a configuration profile for joy message routing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Profile {
    /// Name of the profile
    pub name: String,
    
    /// Optional button index that must be pressed to enable output
    pub enable_button: Option<usize>,
    
    /// Axis to output field mappings
    pub axis_mappings: Vec<AxisMapping>,
    
    /// Button to action mappings
    pub button_mappings: Vec<ButtonMapping>,
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
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputField {
    LinearX,
    LinearY,
    LinearZ,
    AngularX,
    AngularY,
    AngularZ,
}

/// Enum representing different action types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ActionType {
    /// Publish a Twist message with fixed values
    PublishTwist {
        linear_x: f64,
        linear_y: f64,
        linear_z: f64,
        angular_x: f64,
        angular_y: f64,
        angular_z: f64,
    },
    
    /// Call a service (future implementation)
    CallService {
        service_name: String,
        service_type: String,
        // Additional parameters can be added here
    },
}

impl Profile {
    /// Creates a new empty profile
    pub fn new(name: String) -> Self {
        Self {
            name,
            enable_button: None,
            axis_mappings: Vec::new(),
            button_mappings: Vec::new(),
        }
    }
    
    /// Validates the profile configuration
    pub fn validate(&self) -> Result<(), String> {
        // Check for duplicate axis mappings
        let mut used_axes = std::collections::HashSet::new();
        for mapping in &self.axis_mappings {
            if !used_axes.insert(mapping.joy_axis) {
                return Err(format!("Duplicate mapping for axis {}", mapping.joy_axis));
            }
        }
        
        // Check for duplicate button mappings
        let mut used_buttons = std::collections::HashSet::new();
        for mapping in &self.button_mappings {
            if !used_buttons.insert(mapping.button) {
                return Err(format!("Duplicate mapping for button {}", mapping.button));
            }
        }
        
        // Validate scale and deadzone values
        for mapping in &self.axis_mappings {
            if mapping.deadzone < 0.0 {
                return Err(format!("Negative deadzone for axis {}", mapping.joy_axis));
            }
            if mapping.scale == 0.0 {
                return Err(format!("Zero scale for axis {}", mapping.joy_axis));
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
}