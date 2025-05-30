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
            "publish_twist_field" => {
                let field = params_guard
                    .get_parameter(&format!("{}.field", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .unwrap_or_else(|| "linear_x".to_string());
                ActionType::PublishTwistField { field }
            }
            "publish_bool" => {
                let topic = params_guard
                    .get_parameter(&format!("{}.topic", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .unwrap_or_else(|| format!("bool_{}", source_index));
                let value = params_guard
                    .get_parameter(&format!("{}.value", prefix))
                    .and_then(|p| {
                        if let Value::Bool(v) = p.value {
                            Some(v)
                        } else {
                            None
                        }
                    })
                    .unwrap_or(true);
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

                ActionType::PublishBool { topic, value, once }
            }
            "publish_int32" => {
                let topic = params_guard
                    .get_parameter(&format!("{}.topic", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .unwrap_or_else(|| format!("int32_{}", source_index));
                let value = params_guard
                    .get_parameter(&format!("{}.value", prefix))
                    .and_then(|p| {
                        if let Value::I64(v) = p.value {
                            Some(v as i32)
                        } else {
                            None
                        }
                    })
                    .unwrap_or(0);
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

                ActionType::PublishInt32 { topic, value, once }
            }
            "publish_float64" => {
                let topic = params_guard
                    .get_parameter(&format!("{}.topic", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .unwrap_or_else(|| format!("float64_{}", source_index));
                let value = params_guard
                    .get_parameter(&format!("{}.value", prefix))
                    .and_then(|p| {
                        if let Value::F64(v) = p.value {
                            Some(v)
                        } else {
                            None
                        }
                    })
                    .unwrap_or(0.0);
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

                ActionType::PublishFloat64 { topic, value, once }
            }
            "publish_string" => {
                let topic = params_guard
                    .get_parameter(&format!("{}.topic", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .unwrap_or_else(|| format!("string_{}", source_index));
                let value = params_guard
                    .get_parameter(&format!("{}.value", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
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

                ActionType::PublishString { topic, value, once }
            }
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
                    .unwrap_or_else(|| format!("/service_{}", source_index));
                let service_type = params_guard
                    .get_parameter(&format!("{}.service_type", prefix))
                    .and_then(|p| {
                        if let Value::String(v) = &p.value {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .unwrap_or_else(|| "std_srvs/srv/Trigger".to_string());
                ActionType::CallService {
                    service_name,
                    service_type,
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
