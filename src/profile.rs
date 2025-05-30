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
                    .unwrap_or(true);
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
