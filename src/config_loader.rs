use crate::config::{Profile, AxisMapping, ButtonMapping, OutputField, ActionType};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;
use anyhow::{Result, Context};

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
    
    /// Axis mappings
    #[serde(default)]
    pub axis_mappings: Vec<AxisMappingConfig>,
    
    /// Button mappings
    #[serde(default)]
    pub button_mappings: Vec<ButtonMappingConfig>,
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
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = std::fs::read_to_string(&path)
            .with_context(|| format!("Failed to read config file: {:?}", path.as_ref()))?;
        
        let config: Self = serde_yaml::from_str(&content)
            .with_context(|| format!("Failed to parse YAML from: {:?}", path.as_ref()))?;
        
        config.validate()?;
        Ok(config)
    }
    
    /// Validate the configuration
    pub fn validate(&self) -> Result<()> {
        if self.profiles.is_empty() {
            anyhow::bail!("Configuration must contain at least one profile");
        }
        
        if !self.profiles.contains_key(&self.default_profile) {
            anyhow::bail!("Default profile '{}' not found in profiles", self.default_profile);
        }
        
        for (name, profile) in &self.profiles {
            profile.validate()
                .with_context(|| format!("Invalid profile '{}'", name))?;
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
    fn validate(&self) -> Result<()> {
        // Validate axis mappings
        for (i, mapping) in self.axis_mappings.iter().enumerate() {
            if mapping.scale == 0.0 {
                anyhow::bail!("Axis mapping {} has zero scale", i);
            }
            if mapping.deadzone < 0.0 {
                anyhow::bail!("Axis mapping {} has negative deadzone", i);
            }
            // Validate output field
            OutputField::from_str(&mapping.output_field)
                .with_context(|| format!("Invalid output field in axis mapping {}", i))?;
        }
        
        Ok(())
    }
    
    /// Convert to internal Profile structure
    fn to_profile(&self, name: &str) -> Profile {
        let mut profile = Profile::new(name.to_string());
        
        // Set enable button
        if self.enable_button >= 0 {
            profile.enable_button = Some(self.enable_button as usize);
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
                },
                ActionConfig::CallService {
                    service_name,
                    service_type,
                } => ActionType::CallService {
                    service_name: service_name.clone(),
                    service_type: service_type.clone(),
                },
            };
            
            profile.button_mappings.push(ButtonMapping {
                button: mapping.button,
                action,
            });
        }
        
        profile
    }
}

impl OutputField {
    /// Parse from string representation
    fn from_str(s: &str) -> Result<Self> {
        match s {
            "linear_x" => Ok(OutputField::LinearX),
            "linear_y" => Ok(OutputField::LinearY),
            "linear_z" => Ok(OutputField::LinearZ),
            "angular_x" => Ok(OutputField::AngularX),
            "angular_y" => Ok(OutputField::AngularY),
            "angular_z" => Ok(OutputField::AngularZ),
            _ => anyhow::bail!("Unknown output field: {}", s),
        }
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