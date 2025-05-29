use crate::config::{ActionType, InputMapping, InputSource, OutputField, Profile};
use crate::joy_msg_tracker::JoyMsgTracker;
use anyhow::{Result, anyhow};
use safe_drive::parameter::{ParameterServer, Value};

pub fn load_profile_from_params(params: &ParameterServer) -> Result<Profile> {
    let params_guard = params.params.read();
    
    let profile_name = match params_guard.get_parameter("profile_name") {
        Some(param) => match &param.value {
            Value::String(s) => s.clone(),
            _ => return Err(anyhow!("profile_name must be a string")),
        },
        None => return Err(anyhow!("profile_name parameter not found")),
    };
    
    let mut profile = Profile::new(profile_name);
    
    if let Some(param) = params_guard.get_parameter("enable_button") {
        if let Value::I64(button) = param.value {
            if button >= 0 {
                profile.enable_button = Some(button as usize);
            }
        }
    }
    
    if let Some(param) = params_guard.get_parameter("enable_buttons") {
        if let Value::VecI64(buttons) = &param.value {
            let button_vec: Vec<usize> = buttons
                .iter()
                .filter_map(|&b| if b >= 0 { Some(b as usize) } else { None })
                .collect();
            if !button_vec.is_empty() {
                profile.enable_buttons = Some(button_vec);
            }
        }
    }
    
    let mut mapping_idx = 0;
    loop {
        let prefix = format!("input_mappings.{}", mapping_idx);
        
        if params_guard.get_parameter(&format!("{}.source_type", prefix)).is_none() {
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
            "publish_twist_field" => {
                let field_str = params_guard
                    .get_parameter(&format!("{}.field", prefix))
                    .and_then(|p| if let Value::String(v) = &p.value { Some(v.as_str()) } else { None })
                    .unwrap_or("linear_x");
                let field = OutputField::from_str(field_str)?;
                ActionType::PublishTwistField { field }
            }
            "publish_bool" => {
                let topic = params_guard
                    .get_parameter(&format!("{}.topic", prefix))
                    .and_then(|p| if let Value::String(v) = &p.value { Some(v.clone()) } else { None })
                    .unwrap_or_else(|| format!("bool_{}", source_index));
                let value = params_guard
                    .get_parameter(&format!("{}.value", prefix))
                    .and_then(|p| if let Value::Bool(v) = p.value { Some(v) } else { None })
                    .unwrap_or(true);
                let once = params_guard
                    .get_parameter(&format!("{}.once", prefix))
                    .and_then(|p| if let Value::Bool(v) = p.value { Some(v) } else { None })
                    .unwrap_or(true);
                    
                ActionType::PublishBool { topic, value, once }
            }
            "publish_int32" => {
                let topic = params_guard
                    .get_parameter(&format!("{}.topic", prefix))
                    .and_then(|p| if let Value::String(v) = &p.value { Some(v.clone()) } else { None })
                    .unwrap_or_else(|| format!("int32_{}", source_index));
                let value = params_guard
                    .get_parameter(&format!("{}.value", prefix))
                    .and_then(|p| if let Value::I64(v) = p.value { Some(v as i32) } else { None })
                    .unwrap_or(0);
                let once = params_guard
                    .get_parameter(&format!("{}.once", prefix))
                    .and_then(|p| if let Value::Bool(v) = p.value { Some(v) } else { None })
                    .unwrap_or(true);
                    
                ActionType::PublishInt32 { topic, value, once }
            }
            "publish_float64" => {
                let topic = params_guard
                    .get_parameter(&format!("{}.topic", prefix))
                    .and_then(|p| if let Value::String(v) = &p.value { Some(v.clone()) } else { None })
                    .unwrap_or_else(|| format!("float64_{}", source_index));
                let value = params_guard
                    .get_parameter(&format!("{}.value", prefix))
                    .and_then(|p| if let Value::F64(v) = p.value { Some(v) } else { None })
                    .unwrap_or(0.0);
                let once = params_guard
                    .get_parameter(&format!("{}.once", prefix))
                    .and_then(|p| if let Value::Bool(v) = p.value { Some(v) } else { None })
                    .unwrap_or(true);
                    
                ActionType::PublishFloat64 { topic, value, once }
            }
            "publish_string" => {
                let topic = params_guard
                    .get_parameter(&format!("{}.topic", prefix))
                    .and_then(|p| if let Value::String(v) = &p.value { Some(v.clone()) } else { None })
                    .unwrap_or_else(|| format!("string_{}", source_index));
                let value = params_guard
                    .get_parameter(&format!("{}.value", prefix))
                    .and_then(|p| if let Value::String(v) = &p.value { Some(v.clone()) } else { None })
                    .unwrap_or_default();
                let once = params_guard
                    .get_parameter(&format!("{}.once", prefix))
                    .and_then(|p| if let Value::Bool(v) = p.value { Some(v) } else { None })
                    .unwrap_or(true);
                    
                ActionType::PublishString { topic, value, once }
            }
            "call_service" => {
                let service_name = params_guard
                    .get_parameter(&format!("{}.service_name", prefix))
                    .and_then(|p| if let Value::String(v) = &p.value { Some(v.clone()) } else { None })
                    .unwrap_or_else(|| format!("/service_{}", source_index));
                let service_type = params_guard
                    .get_parameter(&format!("{}.service_type", prefix))
                    .and_then(|p| if let Value::String(v) = &p.value { Some(v.clone()) } else { None })
                    .unwrap_or_else(|| "std_srvs/srv/Trigger".to_string());
                let once = params_guard
                    .get_parameter(&format!("{}.once", prefix))
                    .and_then(|p| if let Value::Bool(v) = p.value { Some(v) } else { None })
                    .unwrap_or(true);
                    
                ActionType::CallService {
                    service_name,
                    service_type,
                    once,
                }
            }
            "no_action" => ActionType::NoAction,
            _ => continue,
        };
        
        let scale = params_guard
            .get_parameter(&format!("{}.scale", prefix))
            .and_then(|p| if let Value::F64(v) = p.value { Some(v) } else { None })
            .unwrap_or(1.0);
        let offset = params_guard
            .get_parameter(&format!("{}.offset", prefix))
            .and_then(|p| if let Value::F64(v) = p.value { Some(v) } else { None })
            .unwrap_or(0.0);
        let deadzone = params_guard
            .get_parameter(&format!("{}.deadzone", prefix))
            .and_then(|p| if let Value::F64(v) = p.value { Some(v) } else { None })
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
        if button >= joy_tracker.button_count() || !joy_tracker.is_pressed(button) {
            return false;
        }
    }
    
    if let Some(ref buttons) = profile.enable_buttons {
        let any_pressed = buttons
            .iter()
            .any(|&button| button < joy_tracker.button_count() && joy_tracker.is_pressed(button));
        if !any_pressed {
            return false;
        }
    }
    
    true
}