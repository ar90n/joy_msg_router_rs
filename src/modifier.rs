use crate::config::{
    ActionType, InputMapping, InputSource, InputSourceType, ModifierTarget, SourceTarget,
};
use crate::joy_msg_tracker::JoyMsgTracker;

/// Represents an active modifier and its strength
#[derive(Debug, Clone)]
pub struct ActiveModifier {
    /// The modifier mapping
    pub mapping: InputMapping,
    /// The strength of the modifier (0.0 to 1.0 for gradual modifiers)
    pub strength: f64,
}

/// Collects all active modifiers from the input mappings
pub fn collect_active_modifiers(
    mappings: &[InputMapping],
    tracker: &JoyMsgTracker,
) -> Vec<ActiveModifier> {
    let mut active_modifiers = Vec::new();

    for mapping in mappings {
        if let ActionType::Modifier {
            apply_gradually, ..
        } = &mapping.action
        {
            let (is_active, strength) = match mapping.source {
                InputSource::Axis(idx) => {
                    let value = tracker.get_axis(idx).unwrap_or(0.0) as f64;
                    let processed = mapping.process_value(value);
                    if *apply_gradually {
                        // For gradual modifiers, strength is the processed value
                        (processed.abs() > 0.0, processed.abs())
                    } else {
                        // For non-gradual axis modifiers, it's on/off based on deadzone
                        (processed.abs() > 0.0, 1.0)
                    }
                }
                InputSource::Button(idx) => {
                    let pressed = tracker.is_pressed(idx);
                    (pressed, if pressed { 1.0 } else { 0.0 })
                }
            };

            if is_active {
                active_modifiers.push(ActiveModifier {
                    mapping: mapping.clone(),
                    strength,
                });
            }
        }
    }

    active_modifiers
}

/// Apply modifiers to a mapping, returning a modified copy
pub fn apply_modifiers_to_mapping(
    mapping: &InputMapping,
    active_modifiers: &[ActiveModifier],
    all_mappings: &[InputMapping],
) -> InputMapping {
    let mut modified_mapping = mapping.clone();

    for modifier in active_modifiers {
        if let ActionType::Modifier {
            targets,
            scale_multiplier,
            offset_delta,
            deadzone_override,
            ..
        } = &modifier.mapping.action
        {
            // Check if this modifier targets our mapping
            if is_mapping_targeted(mapping, targets, all_mappings) {
                // Apply modifications based on modifier strength
                if let Some(scale_mult) = scale_multiplier {
                    // Scale is multiplicative
                    let effective_multiplier = 1.0 + (scale_mult - 1.0) * modifier.strength;
                    modified_mapping.scale *= effective_multiplier;
                }

                if let Some(offset_d) = offset_delta {
                    // Offset is additive
                    modified_mapping.offset += offset_d * modifier.strength;
                }

                if let Some(deadzone_o) = deadzone_override {
                    // Deadzone takes the minimum (most restrictive)
                    // Only apply if modifier is at full strength for consistency
                    if modifier.strength >= 1.0 {
                        modified_mapping.deadzone = modified_mapping.deadzone.min(*deadzone_o);
                    }
                }
            }
        }
    }

    modified_mapping
}

/// Check if a mapping is targeted by the given targets
fn is_mapping_targeted(
    mapping: &InputMapping,
    targets: &[ModifierTarget],
    _all_mappings: &[InputMapping],
) -> bool {
    for target in targets {
        match target {
            ModifierTarget::MappingId { mapping_id } => {
                if let Some(id) = &mapping.id {
                    if id == mapping_id {
                        return true;
                    }
                }
            }
            ModifierTarget::Source { source } => {
                if does_source_match(&mapping.source, source) {
                    return true;
                }
            }
        }
    }
    false
}

/// Check if an InputSource matches a SourceTarget
fn does_source_match(input_source: &InputSource, target: &SourceTarget) -> bool {
    match (input_source, target.source_type) {
        (InputSource::Axis(idx), InputSourceType::Axis) => *idx == target.source_index,
        (InputSource::Button(idx), InputSourceType::Button) => *idx == target.source_index,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{ActionType, InputMapping, InputSource};

    #[test]
    fn test_collect_active_button_modifiers() {
        let mappings = vec![InputMapping {
            id: Some("test".to_string()),
            source: InputSource::Button(0),
            action: ActionType::Modifier {
                targets: vec![],
                scale_multiplier: Some(2.0),
                offset_delta: None,
                deadzone_override: None,
                apply_gradually: false,
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.0,
        }];

        let mut tracker = JoyMsgTracker::new();

        // Button not pressed
        tracker.update_buttons(&[0]);
        let modifiers = collect_active_modifiers(&mappings, &tracker);
        assert_eq!(modifiers.len(), 0);

        // Button pressed
        tracker.update_buttons(&[1]);
        let modifiers = collect_active_modifiers(&mappings, &tracker);
        assert_eq!(modifiers.len(), 1);
        assert_eq!(modifiers[0].strength, 1.0);
    }

    #[test]
    fn test_apply_scale_modifier() {
        let target_mapping = InputMapping {
            id: Some("forward".to_string()),
            source: InputSource::Axis(1),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
            },
            scale: 0.5,
            offset: 0.1,
            deadzone: 0.15,
        };

        let modifier = ActiveModifier {
            mapping: InputMapping {
                id: None,
                source: InputSource::Button(5),
                action: ActionType::Modifier {
                    targets: vec![ModifierTarget::MappingId {
                        mapping_id: "forward".to_string(),
                    }],
                    scale_multiplier: Some(2.0),
                    offset_delta: Some(0.2),
                    deadzone_override: Some(0.05),
                    apply_gradually: false,
                },
                scale: 1.0,
                offset: 0.0,
                deadzone: 0.0,
            },
            strength: 1.0,
        };

        let modified =
            apply_modifiers_to_mapping(&target_mapping, &[modifier], &[target_mapping.clone()]);

        assert_eq!(modified.scale, 1.0); // 0.5 * 2.0
        assert_eq!(modified.offset, 0.3); // 0.1 + 0.2
        assert_eq!(modified.deadzone, 0.05); // min(0.15, 0.05)
    }

    #[test]
    fn test_gradual_modifier() {
        let target_mapping = InputMapping {
            id: None,
            source: InputSource::Axis(1),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.1,
        };

        let modifier = ActiveModifier {
            mapping: InputMapping {
                id: None,
                source: InputSource::Axis(6),
                action: ActionType::Modifier {
                    targets: vec![ModifierTarget::Source {
                        source: SourceTarget {
                            source_type: InputSourceType::Axis,
                            source_index: 1,
                        },
                    }],
                    scale_multiplier: Some(3.0),
                    offset_delta: None,
                    deadzone_override: None,
                    apply_gradually: true,
                },
                scale: 1.0,
                offset: 0.0,
                deadzone: 0.0,
            },
            strength: 0.5, // Half strength
        };

        let modified =
            apply_modifiers_to_mapping(&target_mapping, &[modifier], &[target_mapping.clone()]);

        // With 50% strength and 3.0 multiplier: 1.0 + (3.0 - 1.0) * 0.5 = 2.0
        assert_eq!(modified.scale, 2.0);
    }
}
