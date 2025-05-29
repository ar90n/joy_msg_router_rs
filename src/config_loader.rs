use crate::config::{Profile, AxisMapping, ButtonMapping, OutputField, ActionType};
use crate::error::{JoyRouterError, JoyRouterResult};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

/// Top-level configuration structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Configuration {
    /// Default profile to use
    pub default_profile: String,
    
    /// Map of profile names to profiles
    pub profiles: HashMap<String, ProfileConfig>,
}

/// Profile configuration as stored in YAML
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileConfig {
    /// Optional enable button
    #[serde(default = "default_enable_button")]
    pub enable_button: i64,
    
    /// Optional list of enable buttons (OR logic)
    #[serde(default)]
    pub enable_buttons: Vec<usize>,
    
    /// Axis mappings
    #[serde(default)]
    pub axis_mappings: Vec<AxisMappingConfig>,
    
    /// Button mappings
    #[serde(default)]
    pub button_mappings: Vec<ButtonMappingConfig>,
    
    /// Macro definitions available in this profile
    #[serde(default)]
    pub macros: std::collections::HashMap<String, crate::config::MacroDefinition>,
}

/// Axis mapping configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AxisMappingConfig {
    pub joy_axis: usize,
    pub output_field: String,
    pub scale: f64,
    #[serde(default)]
    pub offset: f64,
    pub deadzone: f64,
}

/// Button mapping configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ButtonMappingConfig {
    pub button: usize,
    pub action: ActionConfig,
}

/// Action configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ActionConfig {
    PublishTwist {
        linear_x: f64,
        linear_y: f64,
        linear_z: f64,
        angular_x: f64,
        angular_y: f64,
        angular_z: f64,
    },
    CallService {
        service_name: String,
        service_type: String,
    },
}

fn default_enable_button() -> i64 {
    -1
}

impl Configuration {
    /// Load configuration from a YAML file
    pub fn from_file<P: AsRef<Path>>(path: P) -> JoyRouterResult<Self> {
        let content = std::fs::read_to_string(&path)
            .map_err(|e| JoyRouterError::ConfigError(
                format!("Failed to read config file {:?}: {}", path.as_ref(), e)
            ))?;
        
        let config: Self = serde_yaml::from_str(&content)
            .map_err(|e| JoyRouterError::ConfigError(
                format!("Failed to parse YAML from {:?}: {}", path.as_ref(), e)
            ))?;
        
        config.validate()?;
        Ok(config)
    }
    
    /// Validate the configuration
    pub fn validate(&self) -> JoyRouterResult<()> {
        if self.profiles.is_empty() {
            return Err(JoyRouterError::ConfigError(
                "Configuration must contain at least one profile".to_string()
            ));
        }
        
        if !self.profiles.contains_key(&self.default_profile) {
            return Err(JoyRouterError::ConfigError(
                format!("Default profile '{}' not found in profiles", self.default_profile)
            ));
        }
        
        for (name, profile) in &self.profiles {
            profile.validate()
                .map_err(|e| JoyRouterError::ConfigError(
                    format!("Invalid profile '{}': {}", name, e)
                ))?;
        }
        
        Ok(())
    }
    
    /// Get a profile by name
    pub fn get_profile(&self, name: &str) -> Option<Profile> {
        self.profiles.get(name).map(|config| config.to_profile(name))
    }
    
    /// Get the default profile
    pub fn get_default_profile(&self) -> Profile {
        self.get_profile(&self.default_profile)
            .expect("Default profile should exist after validation")
    }
}

impl ProfileConfig {
    /// Validate the profile configuration
    fn validate(&self) -> JoyRouterResult<()> {
        // Validate axis mappings
        for (i, mapping) in self.axis_mappings.iter().enumerate() {
            if mapping.scale == 0.0 {
                return Err(JoyRouterError::ConfigError(
                    format!("Axis mapping {} has zero scale", i)
                ));
            }
            if mapping.deadzone < 0.0 {
                return Err(JoyRouterError::ConfigError(
                    format!("Axis mapping {} has negative deadzone", i)
                ));
            }
            // Validate output field
            OutputField::from_str(&mapping.output_field)
                .map_err(|e| JoyRouterError::ConfigError(
                    format!("Invalid output field in axis mapping {}: {}", i, e)
                ))?;
        }
        
        Ok(())
    }
    
    /// Convert to internal Profile structure
    fn to_profile(&self, name: &str) -> Profile {
        let mut profile = Profile::new(name.to_string());
        
        // Set enable button(s)
        if self.enable_button >= 0 {
            profile.enable_button = Some(self.enable_button as usize);
        }
        
        // Set multiple enable buttons if specified
        if !self.enable_buttons.is_empty() {
            profile.enable_buttons = Some(self.enable_buttons.clone());
        }
        
        // Convert axis mappings
        for mapping in &self.axis_mappings {
            if let Ok(output_field) = OutputField::from_str(&mapping.output_field) {
                profile.axis_mappings.push(AxisMapping {
                    joy_axis: mapping.joy_axis,
                    output_field,
                    scale: mapping.scale,
                    offset: mapping.offset,
                    deadzone: mapping.deadzone,
                });
            }
        }
        
        // Convert button mappings
        for mapping in &self.button_mappings {
            let action = match &mapping.action {
                ActionConfig::PublishTwist {
                    linear_x,
                    linear_y,
                    linear_z,
                    angular_x,
                    angular_y,
                    angular_z,
                } => ActionType::PublishTwist {
                    linear_x: *linear_x,
                    linear_y: *linear_y,
                    linear_z: *linear_z,
                    angular_x: *angular_x,
                    angular_y: *angular_y,
                    angular_z: *angular_z,
                    once: false, // Default to continuous
                },
                ActionConfig::CallService {
                    service_name,
                    service_type,
                } => ActionType::CallService {
                    service_name: service_name.clone(),
                    service_type: service_type.clone(),
                    once: true, // Default to single call
                },
            };
            
            profile.button_mappings.push(ButtonMapping {
                button: mapping.button,
                action,
            });
        }
        
        // Copy macros
        profile.macros = self.macros.clone();
        
        profile
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_parse_valid_config() {
        let yaml = r#"
default_profile: "test"
profiles:
  test:
    enable_button: 4
    axis_mappings:
      - joy_axis: 1
        output_field: linear_x
        scale: 0.5
        deadzone: 0.1
"#;
        
        let config: Configuration = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.default_profile, "test");
        assert!(config.profiles.contains_key("test"));
        
        let profile_config = &config.profiles["test"];
        assert_eq!(profile_config.enable_button, 4);
        assert_eq!(profile_config.axis_mappings.len(), 1);
    }
    
    #[test]
    fn test_validate_missing_default_profile() {
        let yaml = r#"
default_profile: "missing"
profiles:
  test:
    enable_button: -1
"#;
        
        let config: Configuration = serde_yaml::from_str(yaml).unwrap();
        assert!(config.validate().is_err());
    }
    
    #[test]
    fn test_convert_to_profile() {
        let yaml = r#"
default_profile: "test"
profiles:
  test:
    enable_button: 4
    axis_mappings:
      - joy_axis: 1
        output_field: linear_x
        scale: 0.5
        offset: 0.1
        deadzone: 0.1
    button_mappings:
      - button: 0
        action:
          type: publish_twist
          linear_x: 1.0
          linear_y: 0.0
          linear_z: 0.0
          angular_x: 0.0
          angular_y: 0.0
          angular_z: 0.0
"#;
        
        let config: Configuration = serde_yaml::from_str(yaml).unwrap();
        let profile = config.get_default_profile();
        
        assert_eq!(profile.name, "test");
        assert_eq!(profile.enable_button, Some(4));
        assert_eq!(profile.axis_mappings.len(), 1);
        assert_eq!(profile.button_mappings.len(), 1);
    }
}