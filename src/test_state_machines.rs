use crate::config::*;
use crate::state_machine::*;
use crate::state_machine_manager::*;
use crate::command_queue::CommandQueue;
use crate::button_tracker::ButtonTracker;
use std::sync::Arc;
use std::collections::HashMap;

#[cfg(test)]
mod tests {
    use super::*;
    
    fn create_weapon_system_state_machine() -> StateMachineDefinition {
        let mut states = HashMap::new();
        
        // Safe state - weapons disabled
        states.insert("safe".to_string(), StateDefinition {
            name: "safe".to_string(),
            description: "Weapons are disabled and safe".to_string(),
            on_enter: vec![
                ActionType::PublishBool {
                    topic: "/weapon/armed".to_string(),
                    value: false,
                    once: true,
                }
            ],
            on_update: vec![],
            on_exit: vec![],
            transitions: vec![
                StateTransition {
                    target_state: "arming".to_string(),
                    condition: TransitionCondition::ButtonCombo {
                        buttons: vec![0, 1], // Both buttons must be pressed
                        require_all: true,
                    },
                    delay_ms: 0,
                    priority: 1,
                }
            ],
            timeout_ms: None,
            timeout_target: None,
        });
        
        // Arming state - waiting for confirmation
        states.insert("arming".to_string(), StateDefinition {
            name: "arming".to_string(),
            description: "Waiting for arming confirmation".to_string(),
            on_enter: vec![
                ActionType::PublishString {
                    topic: "/weapon/status".to_string(),
                    value: "ARMING - Press button 2 to confirm".to_string(),
                    once: true,
                }
            ],
            on_update: vec![],
            on_exit: vec![],
            transitions: vec![
                StateTransition {
                    target_state: "armed".to_string(),
                    condition: TransitionCondition::ButtonPress { button: 2 },
                    delay_ms: 0,
                    priority: 1,
                },
                StateTransition {
                    target_state: "safe".to_string(),
                    condition: TransitionCondition::ButtonPress { button: 3 },
                    delay_ms: 0,
                    priority: 2, // Higher priority than timeout
                }
            ],
            timeout_ms: Some(5000), // Auto-return to safe after 5 seconds
            timeout_target: Some("safe".to_string()),
        });
        
        // Armed state - ready to fire
        states.insert("armed".to_string(), StateDefinition {
            name: "armed".to_string(),
            description: "Weapons are armed and ready".to_string(),
            on_enter: vec![
                ActionType::PublishBool {
                    topic: "/weapon/armed".to_string(),
                    value: true,
                    once: true,
                },
                ActionType::PublishString {
                    topic: "/weapon/status".to_string(),
                    value: "ARMED - Ready to fire".to_string(),
                    once: true,
                }
            ],
            on_update: vec![],
            on_exit: vec![
                ActionType::PublishString {
                    topic: "/weapon/status".to_string(),
                    value: "Disarming...".to_string(),
                    once: true,
                }
            ],
            transitions: vec![
                StateTransition {
                    target_state: "firing".to_string(),
                    condition: TransitionCondition::ButtonPress { button: 4 },
                    delay_ms: 0,
                    priority: 1,
                },
                StateTransition {
                    target_state: "safe".to_string(),
                    condition: TransitionCondition::ButtonPress { button: 3 },
                    delay_ms: 0,
                    priority: 1,
                },
                StateTransition {
                    target_state: "safe".to_string(),
                    condition: TransitionCondition::ButtonSequence {
                        buttons: vec![5, 6, 7], // Disarm sequence
                        timeout_ms: 2000,
                    },
                    delay_ms: 0,
                    priority: 2,
                }
            ],
            timeout_ms: Some(10000), // Auto-disarm after 10 seconds
            timeout_target: Some("safe".to_string()),
        });
        
        // Firing state - weapon is firing
        states.insert("firing".to_string(), StateDefinition {
            name: "firing".to_string(),
            description: "Weapon is firing".to_string(),
            on_enter: vec![
                ActionType::PublishBool {
                    topic: "/weapon/fire".to_string(),
                    value: true,
                    once: true,
                },
                ActionType::PublishString {
                    topic: "/weapon/status".to_string(),
                    value: "FIRING!".to_string(),
                    once: true,
                }
            ],
            on_update: vec![],
            on_exit: vec![
                ActionType::PublishBool {
                    topic: "/weapon/fire".to_string(),
                    value: false,
                    once: true,
                }
            ],
            transitions: vec![
                StateTransition {
                    target_state: "armed".to_string(),
                    condition: TransitionCondition::Always,
                    delay_ms: 500, // Fire for 500ms then return to armed
                    priority: 1,
                }
            ],
            timeout_ms: None,
            timeout_target: None,
        });
        
        StateMachineDefinition {
            name: "weapon_system".to_string(),
            description: "Weapon system with safety interlocks".to_string(),
            initial_state: "safe".to_string(),
            states,
        }
    }
    
    fn create_navigation_state_machine() -> StateMachineDefinition {
        let mut states = HashMap::new();
        
        // Manual control mode
        states.insert("manual".to_string(), StateDefinition {
            name: "manual".to_string(),
            description: "Manual control mode".to_string(),
            on_enter: vec![
                ActionType::PublishString {
                    topic: "/nav/mode".to_string(),
                    value: "MANUAL".to_string(),
                    once: true,
                }
            ],
            on_update: vec![],
            on_exit: vec![],
            transitions: vec![
                StateTransition {
                    target_state: "autonomous".to_string(),
                    condition: TransitionCondition::ButtonHeld {
                        button: 8,
                        duration_ms: 2000, // Hold for 2 seconds
                    },
                    delay_ms: 0,
                    priority: 1,
                }
            ],
            timeout_ms: None,
            timeout_target: None,
        });
        
        // Autonomous mode
        states.insert("autonomous".to_string(), StateDefinition {
            name: "autonomous".to_string(),
            description: "Autonomous navigation mode".to_string(),
            on_enter: vec![
                ActionType::PublishString {
                    topic: "/nav/mode".to_string(),
                    value: "AUTONOMOUS".to_string(),
                    once: true,
                },
                ActionType::CallService {
                    service_name: "/nav/start_autonomous".to_string(),
                    service_type: "std_srvs/srv/Trigger".to_string(),
                    once: true,
                }
            ],
            on_update: vec![],
            on_exit: vec![
                ActionType::CallService {
                    service_name: "/nav/stop_autonomous".to_string(),
                    service_type: "std_srvs/srv/Trigger".to_string(),
                    once: true,
                }
            ],
            transitions: vec![
                StateTransition {
                    target_state: "manual".to_string(),
                    condition: TransitionCondition::ButtonPress { button: 9 },
                    delay_ms: 0,
                    priority: 1,
                }
            ],
            timeout_ms: None,
            timeout_target: None,
        });
        
        StateMachineDefinition {
            name: "navigation".to_string(),
            description: "Navigation mode state machine".to_string(),
            initial_state: "manual".to_string(),
            states,
        }
    }
    
    #[test]
    fn test_weapon_system_state_machine() {
        let definition = create_weapon_system_state_machine();
        let mut state_machine = StateMachine::new(definition).unwrap();
        
        // Should start in safe state
        assert_eq!(state_machine.current_state(), "safe");
        
        let mut button_tracker = ButtonTracker::new();
        
        // Test arming sequence - press buttons 0 and 1 together
        button_tracker.update(&[1, 1, 0, 0, 0]); // Buttons 0 and 1 pressed
        let actions = state_machine.update(&button_tracker).unwrap();
        assert_eq!(state_machine.current_state(), "arming");
        assert!(!actions.is_empty()); // Should have on_enter actions
        
        // Test confirmation - press button 2
        button_tracker.update(&[0, 0, 1, 0, 0]); // Button 2 pressed
        let actions = state_machine.update(&button_tracker).unwrap();
        assert_eq!(state_machine.current_state(), "armed");
        assert!(!actions.is_empty()); // Should have on_enter actions
        
        // Test firing - press button 4
        button_tracker.update(&[0, 0, 0, 0, 1]); // Button 4 pressed
        let actions = state_machine.update(&button_tracker).unwrap();
        assert_eq!(state_machine.current_state(), "firing");
        assert!(!actions.is_empty()); // Should have on_enter actions
    }
    
    #[test]
    fn test_state_machine_transitions() {
        let definition = create_weapon_system_state_machine();
        
        // Test all transition condition types
        let button_press = TransitionCondition::ButtonPress { button: 0 };
        let button_release = TransitionCondition::ButtonRelease { button: 1 };
        let button_held = TransitionCondition::ButtonHeld { button: 2, duration_ms: 1000 };
        let button_combo = TransitionCondition::ButtonCombo { 
            buttons: vec![0, 1], 
            require_all: true 
        };
        let button_sequence = TransitionCondition::ButtonSequence { 
            buttons: vec![0, 1, 2], 
            timeout_ms: 2000 
        };
        let always = TransitionCondition::Always;
        
        // Verify conditions are properly structured
        match button_press {
            TransitionCondition::ButtonPress { button } => assert_eq!(button, 0),
            _ => panic!("Expected ButtonPress"),
        }
        
        match button_combo {
            TransitionCondition::ButtonCombo { buttons, require_all } => {
                assert_eq!(buttons, vec![0, 1]);
                assert!(require_all);
            }
            _ => panic!("Expected ButtonCombo"),
        }
        
        match button_sequence {
            TransitionCondition::ButtonSequence { buttons, timeout_ms } => {
                assert_eq!(buttons, vec![0, 1, 2]);
                assert_eq!(timeout_ms, 2000);
            }
            _ => panic!("Expected ButtonSequence"),
        }
        
        match always {
            TransitionCondition::Always => {},
            _ => panic!("Expected Always"),
        }
    }
    
    #[test]
    fn test_state_machine_manager() {
        let mut definitions = HashMap::new();
        definitions.insert("weapon".to_string(), create_weapon_system_state_machine());
        definitions.insert("nav".to_string(), create_navigation_state_machine());
        
        let queue = Arc::new(CommandQueue::new());
        let manager = StateMachineManager::new(definitions, queue);
        
        // Start weapon system
        manager.start_state_machine("weapon").unwrap();
        let status = manager.get_status().unwrap();
        assert_eq!(status.len(), 1);
        assert_eq!(status[0].0, "weapon");
        assert_eq!(status[0].1, "safe");
        
        // Start navigation system
        manager.start_state_machine("nav").unwrap();
        let status = manager.get_status().unwrap();
        assert_eq!(status.len(), 2);
        
        // Test state machine actions
        manager.execute_state_machine_action("weapon", &StateMachineActionType::Reset).unwrap();
        manager.execute_state_machine_action("weapon", &StateMachineActionType::TransitionTo { 
            state: "armed".to_string() 
        }).unwrap();
        
        let status = manager.get_status().unwrap();
        let weapon_status = status.iter().find(|(name, _)| name == "weapon").unwrap();
        assert_eq!(weapon_status.1, "armed");
    }
    
    #[test]
    fn test_state_machine_action_types() {
        let start_action = StateMachineActionType::Start;
        let stop_action = StateMachineActionType::Stop;
        let reset_action = StateMachineActionType::Reset;
        let transition_action = StateMachineActionType::TransitionTo { 
            state: "test_state".to_string() 
        };
        let trigger_action = StateMachineActionType::TriggerCondition { 
            condition_id: "test_condition".to_string() 
        };
        
        // Test action type matching
        match start_action {
            StateMachineActionType::Start => {},
            _ => panic!("Expected Start action"),
        }
        
        match transition_action {
            StateMachineActionType::TransitionTo { state } => {
                assert_eq!(state, "test_state");
            }
            _ => panic!("Expected TransitionTo action"),
        }
        
        match trigger_action {
            StateMachineActionType::TriggerCondition { condition_id } => {
                assert_eq!(condition_id, "test_condition");
            }
            _ => panic!("Expected TriggerCondition action"),
        }
    }
    
    #[test]
    fn test_complex_button_interactions() {
        let definition = create_weapon_system_state_machine();
        let mut state_machine = StateMachine::new(definition).unwrap();
        let mut button_tracker = ButtonTracker::new();
        
        // Test combo press to enter arming state
        button_tracker.update(&[1, 1, 0, 0, 0]); // Press buttons 0 and 1
        state_machine.update(&button_tracker).unwrap();
        assert_eq!(state_machine.current_state(), "arming");
        
        // Confirm arming
        button_tracker.update(&[0, 0, 1, 0, 0]); // Press button 2
        state_machine.update(&button_tracker).unwrap();
        assert_eq!(state_machine.current_state(), "armed");
        
        // Test button sequence for disarming (5, 6, 7)
        button_tracker.update(&[0, 0, 0, 0, 0, 1, 0, 0]); // Press button 5
        state_machine.update(&button_tracker).unwrap();
        
        button_tracker.update(&[0, 0, 0, 0, 0, 0, 1, 0]); // Press button 6
        state_machine.update(&button_tracker).unwrap();
        
        button_tracker.update(&[0, 0, 0, 0, 0, 0, 0, 1]); // Press button 7
        state_machine.update(&button_tracker).unwrap();
        
        // Should transition back to safe after sequence
        assert_eq!(state_machine.current_state(), "safe");
    }
    
    #[test]
    fn test_profile_with_state_machines() {
        let mut profile = Profile::new("state_machine_profile".to_string());
        
        // Add state machines to profile
        let weapon_sm = create_weapon_system_state_machine();
        let nav_sm = create_navigation_state_machine();
        
        profile.state_machines.insert("weapon".to_string(), weapon_sm);
        profile.state_machines.insert("navigation".to_string(), nav_sm);
        
        // Verify state machines are stored
        assert_eq!(profile.state_machines.len(), 2);
        assert!(profile.state_machines.contains_key("weapon"));
        assert!(profile.state_machines.contains_key("navigation"));
        
        let weapon_sm = profile.state_machines.get("weapon").unwrap();
        assert_eq!(weapon_sm.initial_state, "safe");
        assert_eq!(weapon_sm.states.len(), 4); // safe, arming, armed, firing
        
        let nav_sm = profile.state_machines.get("navigation").unwrap();
        assert_eq!(nav_sm.initial_state, "manual");
        assert_eq!(nav_sm.states.len(), 2); // manual, autonomous
    }
    
    #[test]
    fn test_state_machine_action_in_config() {
        let action = ActionType::StateMachineAction {
            state_machine: "weapon_system".to_string(),
            action: StateMachineActionType::Start,
        };
        
        match action {
            ActionType::StateMachineAction { state_machine, action } => {
                assert_eq!(state_machine, "weapon_system");
                match action {
                    StateMachineActionType::Start => {},
                    _ => panic!("Expected Start action"),
                }
            }
            _ => panic!("Expected StateMachineAction"),
        }
    }
}