use crate::config::{ActionType, InputMapping, InputSource, Profile};
use anyhow::{anyhow, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

#[derive(Debug, Deserialize, Serialize)]
struct YamlConfig {
    default_profile: Option<String>,
    profiles: HashMap<String, YamlProfile>,
}

#[derive(Debug, Deserialize, Serialize)]
struct YamlProfile {
    enable_button: Option<usize>,
    input_mappings: Vec<YamlInputMapping>,
}

#[derive(Debug, Deserialize, Serialize)]
struct YamlInputMapping {
    source_type: String,
    source_index: usize,
    scale: Option<f64>,
    offset: Option<f64>,
    deadzone: Option<f64>,
    action: YamlAction,
}

#[derive(Debug, Deserialize, Serialize)]
struct YamlAction {
    #[serde(rename = "type")]
    action_type: String,
    topic: Option<String>,
    message_type: Option<String>,
    field: Option<String>,
    once: Option<bool>,
    service_name: Option<String>,
    service_type: Option<String>,
}

pub fn load_profile_from_yaml_file<P: AsRef<Path>>(
    file_path: P,
    profile_name: Option<&str>,
) -> Result<Profile> {
    let yaml_content = fs::read_to_string(&file_path)
        .map_err(|e| anyhow!("Failed to read YAML file: {}", e))?;
    
    let config: YamlConfig = serde_yaml::from_str(&yaml_content)
        .map_err(|e| anyhow!("Failed to parse YAML: {}", e))?;
    
    let selected_profile_name = profile_name
        .or(config.default_profile.as_deref())
        .ok_or_else(|| anyhow!("No profile specified and no default profile defined"))?;
    
    let yaml_profile = config.profiles
        .get(selected_profile_name)
        .ok_or_else(|| anyhow!("Profile '{}' not found in configuration", selected_profile_name))?;
    
    let mut profile = Profile::new(selected_profile_name.to_string());
    profile.enable_button = yaml_profile.enable_button;
    
    for yaml_mapping in &yaml_profile.input_mappings {
        let source = match yaml_mapping.source_type.as_str() {
            "axis" => InputSource::Axis(yaml_mapping.source_index),
            "button" => InputSource::Button(yaml_mapping.source_index),
            _ => return Err(anyhow!("Invalid source type: {}", yaml_mapping.source_type)),
        };
        
        let action = match yaml_mapping.action.action_type.as_str() {
            "publish" => {
                let topic = yaml_mapping.action.topic
                    .as_ref()
                    .ok_or_else(|| anyhow!("Missing topic for publish action"))?
                    .clone();
                let message_type = yaml_mapping.action.message_type
                    .as_ref()
                    .ok_or_else(|| anyhow!("Missing message_type for publish action"))?
                    .clone();
                let field = yaml_mapping.action.field.clone();
                let once = yaml_mapping.action.once.unwrap_or(false);
                
                ActionType::Publish {
                    topic,
                    message_type,
                    field,
                    once,
                }
            }
            "call_service" => {
                let service_name = yaml_mapping.action.service_name
                    .as_ref()
                    .ok_or_else(|| anyhow!("Missing service_name for call_service action"))?
                    .clone();
                let service_type = yaml_mapping.action.service_type
                    .as_ref()
                    .ok_or_else(|| anyhow!("Missing service_type for call_service action"))?
                    .clone();
                
                ActionType::CallService {
                    service_name,
                    service_type,
                }
            }
            _ => return Err(anyhow!("Invalid action type: {}", yaml_mapping.action.action_type)),
        };
        
        profile.input_mappings.push(InputMapping {
            source,
            action,
            scale: yaml_mapping.scale.unwrap_or(1.0),
            offset: yaml_mapping.offset.unwrap_or(0.0),
            deadzone: yaml_mapping.deadzone.unwrap_or(0.1),
        });
    }
    
    profile.validate()?;
    
    Ok(profile)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;
    
    #[test]
    fn test_load_yaml_profile() {
        let yaml_content = r#"
default_profile: "test"
profiles:
  test:
    enable_button: 4
    input_mappings:
      - source_type: axis
        source_index: 1
        scale: 0.5
        offset: 0.0
        deadzone: 0.1
        action:
          type: publish
          topic: "/cmd_vel"
          message_type: "geometry_msgs/msg/Twist"
          field: "linear.x"
      - source_type: button
        source_index: 0
        action:
          type: call_service
          service_name: "/emergency_stop"
          service_type: "std_srvs/srv/Trigger"
"#;
        
        let mut temp_file = NamedTempFile::new().unwrap();
        write!(temp_file, "{}", yaml_content).unwrap();
        
        let profile = load_profile_from_yaml_file(temp_file.path(), None).unwrap();
        
        assert_eq!(profile.name, "test");
        assert_eq!(profile.enable_button, Some(4));
        assert_eq!(profile.input_mappings.len(), 2);
        
        match &profile.input_mappings[0].source {
            InputSource::Axis(idx) => assert_eq!(*idx, 1),
            _ => panic!("Expected Axis source"),
        }
        
        match &profile.input_mappings[1].action {
            ActionType::CallService { service_name, service_type } => {
                assert_eq!(service_name, "/emergency_stop");
                assert_eq!(service_type, "std_srvs/srv/Trigger");
            }
            _ => panic!("Expected CallService action"),
        }
    }
    
    #[test]
    fn test_load_specific_profile() {
        let yaml_content = r#"
default_profile: "profile1"
profiles:
  profile1:
    input_mappings:
      - source_type: button
        source_index: 0
        action:
          type: publish
          topic: "/test1"
          message_type: "std_msgs/msg/Bool"
  profile2:
    input_mappings:
      - source_type: button
        source_index: 1
        action:
          type: publish
          topic: "/test2"
          message_type: "std_msgs/msg/Bool"
"#;
        
        let mut temp_file = NamedTempFile::new().unwrap();
        write!(temp_file, "{}", yaml_content).unwrap();
        
        let profile = load_profile_from_yaml_file(temp_file.path(), Some("profile2")).unwrap();
        
        assert_eq!(profile.name, "profile2");
        match &profile.input_mappings[0].action {
            ActionType::Publish { topic, .. } => assert_eq!(topic, "/test2"),
            _ => panic!("Expected Publish action"),
        }
    }
}