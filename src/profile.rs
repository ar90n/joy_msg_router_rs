use crate::config::{ActionType, InputMapping, InputSource, Profile};
use crate::joy_msg_tracker::JoyMsgTracker;
use anyhow::{anyhow, Result};
use safe_drive::parameter::{ParameterServer, Value};

pub fn load_profile_from_params(params: &ParameterServer) -> Result<Profile> {
    let params_guard = params.params.read();

    let profile_name = params_guard
        .get_parameter("profile_name")
        .ok_or_else(|| anyhow!("profile_name parameter not found"))
        .and_then(|param| match &param.value {
            Value::String(s) => Ok(s.clone()),
            _ => Err(anyhow!("profile_name must be a string")),
        })?;

    let mut profile = Profile::new(profile_name);
    profile.enable_button =
        params_guard
            .get_parameter("enable_button")
            .and_then(|param| match param.value {
                Value::I64(v) if 0 <= v => Some(v as usize),
                _ => None,
            });

    let mut mapping_idx = 0;
    loop {
        let prefix = format!("input_mappings.{}", mapping_idx);

        if params_guard
            .get_parameter(&format!("{}.source_type", prefix))
            .is_none()
        {
            break;
        }

        let source_type = match params_guard.get_parameter(&format!("{}.source_type", prefix)) {
            Some(param) => match &param.value {
                Value::String(s) => s.as_str(),
                _ => continue,
            },
            None => continue,
        };

        let source_index = match params_guard.get_parameter(&format!("{}.source_index", prefix)) {
            Some(param) => match param.value {
                Value::I64(v) => v as usize,
                _ => continue,
            },
            None => continue,
        };

        let source = match source_type {
            "axis" => InputSource::Axis(source_index),
            "button" => InputSource::Button(source_index),
            _ => continue,
        };

        let action_type = match params_guard.get_parameter(&format!("{}.action_type", prefix)) {
            Some(param) => match &param.value {
                Value::String(s) => s.as_str(),
                _ => continue,
            },
            None => continue,
        };

        let action = match action_type {
            "call_service" => {
                let service_name = params_guard
                    .get_parameter(&format!("{}.service_name", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .ok_or_else(|| anyhow!("Missing service_name for call_service"))?;
                let service_type = params_guard
                    .get_parameter(&format!("{}.service_type", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .ok_or_else(|| anyhow!("Missing service_type for call_service"))?;
                ActionType::CallService {
                    service_name,
                    service_type,
                }
            }
            "publish" => {
                let topic = params_guard
                    .get_parameter(&format!("{}.topic", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .ok_or_else(|| anyhow!("Missing topic for publish"))?;
                let message_type = params_guard
                    .get_parameter(&format!("{}.message_type", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .ok_or_else(|| anyhow!("Missing message_type for publish"))?;
                let field = params_guard
                    .get_parameter(&format!("{}.field", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    });
                let once = params_guard
                    .get_parameter(&format!("{}.once", prefix))
                    .and_then(|p| {
                        if let Value::Bool(v) = p.value {
                            Some(v)
                        } else {
                            None
                        }
                    })
                    .unwrap_or(false);
                ActionType::Publish {
                    topic,
                    message_type,
                    field,
                    once,
                }
            }
            _ => continue,
        };

        let scale = params_guard
            .get_parameter(&format!("{}.scale", prefix))
            .and_then(|p| {
                if let Value::F64(v) = p.value {
                    Some(v)
                } else {
                    None
                }
            })
            .unwrap_or(1.0);
        let offset = params_guard
            .get_parameter(&format!("{}.offset", prefix))
            .and_then(|p| {
                if let Value::F64(v) = p.value {
                    Some(v)
                } else {
                    None
                }
            })
            .unwrap_or(0.0);
        let deadzone = params_guard
            .get_parameter(&format!("{}.deadzone", prefix))
            .and_then(|p| {
                if let Value::F64(v) = p.value {
                    Some(v)
                } else {
                    None
                }
            })
            .unwrap_or(0.1);

        profile.input_mappings.push(InputMapping {
            source,
            action,
            scale,
            offset,
            deadzone,
        });

        mapping_idx += 1;
    }

    profile.validate()?;

    Ok(profile)
}

pub fn is_enabled(profile: &Profile, joy_tracker: &JoyMsgTracker) -> bool {
    if let Some(button) = profile.enable_button {
        return joy_tracker.is_pressed(button);
    }

    // If no enable button is set, profile is enabled by default
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    
    // Mock parameter storage for testing
    struct MockParams {
        params: std::collections::HashMap<String, Value>,
    }
    
    impl MockParams {
        fn new() -> Self {
            Self {
                params: std::collections::HashMap::new(),
            }
        }
        
        fn add(&mut self, name: &str, value: Value) {
            self.params.insert(name.to_string(), value);
        }
        
        fn to_profile(&self) -> Result<Profile> {
            let profile_name = self.params.get("profile_name")
                .and_then(|v| if let Value::String(s) = v { Some(s.clone()) } else { None })
                .ok_or_else(|| anyhow!("profile_name not found"))?;
            
            let mut profile = Profile::new(profile_name);
            
            // Extract enable button
            profile.enable_button = self.params.get("enable_button")
                .and_then(|v| if let Value::I64(i) = v { Some(*i as usize) } else { None });
            
            // Extract mappings
            let mut idx = 0;
            loop {
                let prefix = format!("input_mappings.{}", idx);
                if !self.params.contains_key(&format!("{}.source_type", prefix)) {
                    break;
                }
                
                let source_type = self.params.get(&format!("{}.source_type", prefix))
                    .and_then(|v| if let Value::String(s) = v { Some(s.as_str()) } else { None });
                let source_index = self.params.get(&format!("{}.source_index", prefix))
                    .and_then(|v| if let Value::I64(i) = v { Some(*i as usize) } else { None });
                
                if let (Some(source_type), Some(source_index)) = (source_type, source_index) {
                    let source = match source_type {
                        "axis" => InputSource::Axis(source_index),
                        "button" => InputSource::Button(source_index),
                        _ => continue,
                    };
                    
                    let action_type = self.params.get(&format!("{}.action_type", prefix))
                        .and_then(|v| if let Value::String(s) = v { Some(s.as_str()) } else { None });
                    
                    let action = match action_type {
                        Some("call_service") => {
                            let service_name = self.params.get(&format!("{}.service_name", prefix))
                                .and_then(|v| if let Value::String(s) = v { Some(s.clone()) } else { None })
                                .ok_or_else(|| anyhow!("Missing service_name"))?;
                            let service_type = self.params.get(&format!("{}.service_type", prefix))
                                .and_then(|v| if let Value::String(s) = v { Some(s.clone()) } else { None })
                                .ok_or_else(|| anyhow!("Missing service_type"))?;
                            ActionType::CallService { service_name, service_type }
                        }
                        Some("publish") => {
                            let topic = self.params.get(&format!("{}.topic", prefix))
                                .and_then(|v| if let Value::String(s) = v { Some(s.clone()) } else { None })
                                .ok_or_else(|| anyhow!("Missing topic"))?;
                            let message_type = self.params.get(&format!("{}.message_type", prefix))
                                .and_then(|v| if let Value::String(s) = v { Some(s.clone()) } else { None })
                                .ok_or_else(|| anyhow!("Missing message_type"))?;
                            let field = self.params.get(&format!("{}.field", prefix))
                                .and_then(|v| if let Value::String(s) = v { Some(s.clone()) } else { None });
                            let once = self.params.get(&format!("{}.once", prefix))
                                .and_then(|v| if let Value::Bool(b) = v { Some(*b) } else { None })
                                .unwrap_or(true);
                            ActionType::Publish { topic, message_type, field, once }
                        }
                        _ => continue,
                    };
                    
                    let scale = self.params.get(&format!("{}.scale", prefix))
                        .and_then(|v| if let Value::F64(f) = v { Some(*f) } else { None })
                        .unwrap_or(1.0);
                    let offset = self.params.get(&format!("{}.offset", prefix))
                        .and_then(|v| if let Value::F64(f) = v { Some(*f) } else { None })
                        .unwrap_or(0.0);
                    let deadzone = self.params.get(&format!("{}.deadzone", prefix))
                        .and_then(|v| if let Value::F64(f) = v { Some(*f) } else { None })
                        .unwrap_or(0.1);
                    
                    profile.input_mappings.push(InputMapping {
                        source,
                        action,
                        scale,
                        offset,
                        deadzone,
                    });
                }
                
                idx += 1;
            }
            
            Ok(profile)
        }
    }
    
    #[test]
    fn test_load_profile_with_service_mappings() {
        let mut params = MockParams::new();
        
        // Set profile name
        params.add("profile_name", Value::String("test_profile".to_string()));
        
        // Add a service mapping
        params.add("input_mappings.0.source_type", Value::String("button".to_string()));
        params.add("input_mappings.0.source_index", Value::I64(3));
        params.add("input_mappings.0.action_type", Value::String("call_service".to_string()));
        params.add("input_mappings.0.service_name", Value::String("/reset_system".to_string()));
        params.add("input_mappings.0.service_type", Value::String("std_srvs/srv/Trigger".to_string()));
        params.add("input_mappings.0.scale", Value::F64(1.0));
        
        let profile = params.to_profile().unwrap();
        
        assert_eq!(profile.name, "test_profile");
        assert_eq!(profile.input_mappings.len(), 1);
        
        match &profile.input_mappings[0].action {
            ActionType::CallService { service_name, service_type } => {
                assert_eq!(service_name, "/reset_system");
                assert_eq!(service_type, "std_srvs/srv/Trigger");
            }
            _ => panic!("Expected CallService action"),
        }
    }
    
    #[test]
    fn test_load_profile_mixed_actions() {
        let mut params = MockParams::new();
        
        params.add("profile_name", Value::String("mixed".to_string()));
        params.add("enable_button", Value::I64(4));
        
        // Publish action
        params.add("input_mappings.0.source_type", Value::String("axis".to_string()));
        params.add("input_mappings.0.source_index", Value::I64(1));
        params.add("input_mappings.0.action_type", Value::String("publish".to_string()));
        params.add("input_mappings.0.topic", Value::String("/cmd_vel".to_string()));
        params.add("input_mappings.0.message_type", Value::String("geometry_msgs/msg/Twist".to_string()));
        params.add("input_mappings.0.field", Value::String("linear.x".to_string()));
        params.add("input_mappings.0.scale", Value::F64(0.5));
        params.add("input_mappings.0.deadzone", Value::F64(0.15));
        
        // Service action
        params.add("input_mappings.1.source_type", Value::String("button".to_string()));
        params.add("input_mappings.1.source_index", Value::I64(0));
        params.add("input_mappings.1.action_type", Value::String("call_service".to_string()));
        params.add("input_mappings.1.service_name", Value::String("/emergency_stop".to_string()));
        params.add("input_mappings.1.service_type", Value::String("std_srvs/srv/Empty".to_string()));
        
        let profile = params.to_profile().unwrap();
        
        assert_eq!(profile.enable_button, Some(4));
        assert_eq!(profile.input_mappings.len(), 2);
        
        // Verify publish action
        match &profile.input_mappings[0].source {
            InputSource::Axis(idx) => assert_eq!(*idx, 1),
            _ => panic!("Expected Axis source"),
        }
        
        // Verify service action
        match &profile.input_mappings[1].action {
            ActionType::CallService { service_name, service_type } => {
                assert_eq!(service_name, "/emergency_stop");
                assert_eq!(service_type, "std_srvs/srv/Empty");
            }
            _ => panic!("Expected CallService action"),
        }
    }
    
    #[test]
    fn test_missing_service_type_fails() {
        let mut params = MockParams::new();
        
        params.add("profile_name", Value::String("test".to_string()));
        
        // Add service mapping without service_type
        params.add("input_mappings.0.source_type", Value::String("button".to_string()));
        params.add("input_mappings.0.source_index", Value::I64(0));
        params.add("input_mappings.0.action_type", Value::String("call_service".to_string()));
        params.add("input_mappings.0.service_name", Value::String("/test".to_string()));
        // Missing service_type
        
        let result = params.to_profile();
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("Missing service_type"));
    }
    
    #[test]
    fn test_is_enabled_with_service_actions() {
        let mut profile = Profile::new("test".to_string());
        profile.enable_button = Some(5);
        
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(0),
            action: ActionType::CallService {
                service_name: "/test".to_string(),
                service_type: "std_srvs/srv/Trigger".to_string(),
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        let mut tracker = JoyMsgTracker::new();
        
        // Enable button not pressed
        tracker.update_buttons(&[1, 0, 0, 0, 0, 0]);
        assert!(!is_enabled(&profile, &tracker));
        
        // Enable button pressed
        tracker.update_buttons(&[1, 0, 0, 0, 0, 1]);
        assert!(is_enabled(&profile, &tracker));
    }
}
