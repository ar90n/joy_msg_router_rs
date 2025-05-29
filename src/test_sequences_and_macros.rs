use crate::config::*;
use crate::sequence_executor::SequenceExecutor;
use crate::command_queue::CommandQueue;
use std::sync::Arc;

#[cfg(test)]
mod tests {
    use super::*;
    
    fn create_test_sequence() -> Vec<SequenceStep> {
        vec![
            SequenceStep {
                action: ActionType::PublishTwist {
                    linear_x: 1.0,
                    linear_y: 0.0,
                    linear_z: 0.0,
                    angular_x: 0.0,
                    angular_y: 0.0,
                    angular_z: 0.5,
                    once: true,
                },
                delay_ms: 0,
                duration_ms: 100,
            },
            SequenceStep {
                action: ActionType::PublishBool {
                    topic: "/test_topic".to_string(),
                    value: true,
                    once: true,
                },
                delay_ms: 50,
                duration_ms: 0,
            },
            SequenceStep {
                action: ActionType::NoAction,
                delay_ms: 100,
                duration_ms: 0,
            },
        ]
    }
    
    fn create_test_macro() -> MacroDefinition {
        MacroDefinition {
            name: "test_macro".to_string(),
            description: "A test macro for validation".to_string(),
            parameters: {
                let mut params = std::collections::HashMap::new();
                params.insert("speed".to_string(), MacroParameterDef {
                    param_type: "number".to_string(),
                    default: Some(MacroParameter::Number(1.0)),
                    description: "Movement speed".to_string(),
                });
                params.insert("topic".to_string(), MacroParameterDef {
                    param_type: "string".to_string(),
                    default: Some(MacroParameter::String("/cmd_vel".to_string())),
                    description: "Topic to publish to".to_string(),
                });
                params
            },
            actions: vec![
                SequenceStep {
                    action: ActionType::PublishTwist {
                        linear_x: 2.0,
                        linear_y: 0.0,
                        linear_z: 0.0,
                        angular_x: 0.0,
                        angular_y: 0.0,
                        angular_z: 1.0,
                        once: true,
                    },
                    delay_ms: 0,
                    duration_ms: 500,
                },
                SequenceStep {
                    action: ActionType::NoAction,
                    delay_ms: 0,
                    duration_ms: 0,
                },
            ],
        }
    }
    
    #[test]
    fn test_action_sequence_config() {
        let sequence = create_test_sequence();
        
        let action = ActionType::ActionSequence {
            actions: sequence.clone(),
            once: true,
            repeat: false,
        };
        
        match action {
            ActionType::ActionSequence { actions, once, repeat } => {
                assert_eq!(actions.len(), 3);
                assert!(once);
                assert!(!repeat);
                
                // Check first step
                assert_eq!(actions[0].delay_ms, 0);
                assert_eq!(actions[0].duration_ms, 100);
                match &actions[0].action {
                    ActionType::PublishTwist { linear_x, angular_z, .. } => {
                        assert_eq!(*linear_x, 1.0);
                        assert_eq!(*angular_z, 0.5);
                    }
                    _ => panic!("Expected PublishTwist action"),
                }
                
                // Check second step
                assert_eq!(actions[1].delay_ms, 50);
                assert_eq!(actions[1].duration_ms, 0);
                match &actions[1].action {
                    ActionType::PublishBool { topic, value, .. } => {
                        assert_eq!(topic, "/test_topic");
                        assert!(*value);
                    }
                    _ => panic!("Expected PublishBool action"),
                }
            }
            _ => panic!("Expected ActionSequence"),
        }
    }
    
    #[test]
    fn test_execute_macro_config() {
        let mut parameters = std::collections::HashMap::new();
        parameters.insert("speed".to_string(), MacroParameter::Number(2.5));
        parameters.insert("enabled".to_string(), MacroParameter::Boolean(true));
        
        let action = ActionType::ExecuteMacro {
            macro_name: "test_movement".to_string(),
            parameters,
            once: true,
        };
        
        match action {
            ActionType::ExecuteMacro { macro_name, parameters, once } => {
                assert_eq!(macro_name, "test_movement");
                assert!(once);
                assert_eq!(parameters.len(), 2);
                
                match parameters.get("speed") {
                    Some(MacroParameter::Number(val)) => assert_eq!(*val, 2.5),
                    _ => panic!("Expected speed parameter to be a number"),
                }
                
                match parameters.get("enabled") {
                    Some(MacroParameter::Boolean(val)) => assert!(*val),
                    _ => panic!("Expected enabled parameter to be a boolean"),
                }
            }
            _ => panic!("Expected ExecuteMacro action"),
        }
    }
    
    #[test]
    fn test_macro_definition() {
        let macro_def = create_test_macro();
        
        assert_eq!(macro_def.name, "test_macro");
        assert!(!macro_def.description.is_empty());
        assert_eq!(macro_def.parameters.len(), 2);
        assert_eq!(macro_def.actions.len(), 2);
        
        // Check parameters
        let speed_param = macro_def.parameters.get("speed").unwrap();
        assert_eq!(speed_param.param_type, "number");
        match &speed_param.default {
            Some(MacroParameter::Number(val)) => assert_eq!(*val, 1.0),
            _ => panic!("Expected default speed to be a number"),
        }
        
        let topic_param = macro_def.parameters.get("topic").unwrap();
        assert_eq!(topic_param.param_type, "string");
        match &topic_param.default {
            Some(MacroParameter::String(val)) => assert_eq!(val, "/cmd_vel"),
            _ => panic!("Expected default topic to be a string"),
        }
    }
    
    #[test]
    fn test_macro_parameter_types() {
        // Test different parameter types
        let string_param = MacroParameter::String("test_value".to_string());
        let number_param = MacroParameter::Number(42.5);
        let boolean_param = MacroParameter::Boolean(false);
        let array_param = MacroParameter::Array(vec![
            MacroParameter::Number(1.0),
            MacroParameter::Number(2.0),
            MacroParameter::Number(3.0),
        ]);
        
        match string_param {
            MacroParameter::String(val) => assert_eq!(val, "test_value"),
            _ => panic!("Expected string parameter"),
        }
        
        match number_param {
            MacroParameter::Number(val) => assert_eq!(val, 42.5),
            _ => panic!("Expected number parameter"),
        }
        
        match boolean_param {
            MacroParameter::Boolean(val) => assert!(!val),
            _ => panic!("Expected boolean parameter"),
        }
        
        match array_param {
            MacroParameter::Array(values) => {
                assert_eq!(values.len(), 3);
                for (i, value) in values.iter().enumerate() {
                    match value {
                        MacroParameter::Number(val) => assert_eq!(*val, (i + 1) as f64),
                        _ => panic!("Expected number in array"),
                    }
                }
            }
            _ => panic!("Expected array parameter"),
        }
    }
    
    #[test]
    fn test_sequence_executor_with_macro() {
        let queue = Arc::new(CommandQueue::new());
        let executor = SequenceExecutor::new(queue);
        
        let macro_def = create_test_macro();
        let parameters = std::collections::HashMap::new(); // Use defaults
        
        // Should be able to execute macro (converts to sequence internally)
        let result = executor.execute_macro(&macro_def, &parameters);
        assert!(result.is_ok());
    }
    
    #[test]
    fn test_macro_parameter_validation() {
        let queue = Arc::new(CommandQueue::new());
        let executor = SequenceExecutor::new(queue);
        
        // Create macro with required parameter (no default)
        let mut macro_def = create_test_macro();
        macro_def.parameters.insert("required_param".to_string(), MacroParameterDef {
            param_type: "string".to_string(),
            default: None, // No default value
            description: "Required parameter".to_string(),
        });
        
        let parameters = std::collections::HashMap::new(); // Missing required parameter
        
        // Should fail due to missing required parameter
        let result = executor.execute_macro(&macro_def, &parameters);
        assert!(result.is_err());
        
        // Should succeed with required parameter provided
        let mut parameters_with_required = std::collections::HashMap::new();
        parameters_with_required.insert("required_param".to_string(), MacroParameter::String("value".to_string()));
        let result = executor.execute_macro(&macro_def, &parameters_with_required);
        assert!(result.is_ok());
    }
    
    #[test]
    fn test_profile_with_macros() {
        let mut profile = Profile::new("test_with_macros".to_string());
        
        // Add a macro to the profile
        let macro_def = create_test_macro();
        profile.macros.insert("test_macro".to_string(), macro_def);
        
        // Verify macro is stored
        assert_eq!(profile.macros.len(), 1);
        assert!(profile.macros.contains_key("test_macro"));
        
        let stored_macro = profile.macros.get("test_macro").unwrap();
        assert_eq!(stored_macro.name, "test_macro");
        assert_eq!(stored_macro.actions.len(), 2);
    }
    
    #[test]
    fn test_sequence_step_timing() {
        let step = SequenceStep {
            action: ActionType::PublishTwist {
                linear_x: 1.0,
                linear_y: 0.0,
                linear_z: 0.0,
                angular_x: 0.0,
                angular_y: 0.0,
                angular_z: 0.0,
                once: true,
            },
            delay_ms: 250,
            duration_ms: 1000,
        };
        
        assert_eq!(step.delay_ms, 250);
        assert_eq!(step.duration_ms, 1000);
        
        // Test instantaneous action (duration_ms = 0)
        let instant_step = SequenceStep {
            action: ActionType::NoAction,
            delay_ms: 0,
            duration_ms: 0,
        };
        
        assert_eq!(instant_step.delay_ms, 0);
        assert_eq!(instant_step.duration_ms, 0);
    }
}