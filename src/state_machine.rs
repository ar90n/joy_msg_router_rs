use std::time::{Duration, Instant};
use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use crate::config::ActionType;
use crate::button_tracker::ButtonTracker;
use crate::error::{JoyRouterError, JoyRouterResult};
use safe_drive::logger::Logger;
use safe_drive::{pr_debug, pr_info, pr_warn};

/// Defines a state in the state machine
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StateDefinition {
    /// Name of the state
    pub name: String,
    /// Description of what this state represents
    #[serde(default)]
    pub description: String,
    /// Actions to execute when entering this state
    #[serde(default)]
    pub on_enter: Vec<ActionType>,
    /// Actions to execute while in this state (per update)
    #[serde(default)]
    pub on_update: Vec<ActionType>,
    /// Actions to execute when exiting this state
    #[serde(default)]
    pub on_exit: Vec<ActionType>,
    /// Transitions to other states
    #[serde(default)]
    pub transitions: Vec<StateTransition>,
    /// Whether this state should auto-timeout after a duration
    #[serde(default)]
    pub timeout_ms: Option<u64>,
    /// State to transition to on timeout
    #[serde(default)]
    pub timeout_target: Option<String>,
}

/// Defines a transition between states
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StateTransition {
    /// Target state to transition to
    pub target_state: String,
    /// Condition that triggers this transition
    pub condition: TransitionCondition,
    /// Optional delay before transition occurs
    #[serde(default)]
    pub delay_ms: u64,
    /// Priority of this transition (higher = checked first)
    #[serde(default)]
    pub priority: u32,
}

/// Conditions that can trigger state transitions
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum TransitionCondition {
    /// Single button press
    ButtonPress { button: usize },
    /// Single button release
    ButtonRelease { button: usize },
    /// Button held for duration
    ButtonHeld { button: usize, duration_ms: u64 },
    /// Multiple buttons pressed simultaneously
    ButtonCombo { buttons: Vec<usize>, require_all: bool },
    /// Button sequence (buttons pressed in order)
    ButtonSequence { buttons: Vec<usize>, timeout_ms: u64 },
    /// Always true (for timeout transitions)
    Always,
    /// Custom condition (placeholder for future extension)
    Custom { condition_id: String },
}

/// Runtime state of a state machine
#[derive(Debug)]
pub struct StateMachineState {
    /// Current state name
    pub current_state: String,
    /// When the current state was entered
    pub state_enter_time: Instant,
    /// Pending transition (if any)
    pub pending_transition: Option<PendingTransition>,
    /// Button sequence tracking for sequence conditions
    pub sequence_tracker: ButtonSequenceTracker,
    /// Button hold tracking for held conditions
    pub hold_tracker: ButtonHoldTracker,
}

/// A transition that is delayed
#[derive(Debug, Clone)]
pub struct PendingTransition {
    pub target_state: String,
    pub trigger_time: Instant,
}

/// Tracks button sequences for sequence conditions
#[derive(Debug)]
pub struct ButtonSequenceTracker {
    /// Current sequence being tracked
    pub current_sequence: Vec<usize>,
    /// When the sequence started
    pub sequence_start: Option<Instant>,
    /// Expected sequence
    pub expected_sequence: Option<Vec<usize>>,
    /// Timeout for sequence completion
    pub timeout_ms: u64,
}

/// Tracks button hold durations
#[derive(Debug)]
pub struct ButtonHoldTracker {
    /// Button hold start times
    pub hold_starts: HashMap<usize, Instant>,
}

/// State machine definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StateMachineDefinition {
    /// Name of the state machine
    pub name: String,
    /// Description
    #[serde(default)]
    pub description: String,
    /// Initial state
    pub initial_state: String,
    /// All states in the machine
    pub states: HashMap<String, StateDefinition>,
}

/// Manages a running state machine instance
pub struct StateMachine {
    /// State machine definition
    definition: StateMachineDefinition,
    /// Current runtime state
    state: StateMachineState,
    /// Logger for state machine events
    logger: Logger,
}

impl StateMachine {
    /// Create a new state machine instance
    pub fn new(definition: StateMachineDefinition) -> JoyRouterResult<Self> {
        // Validate that initial state exists
        if !definition.states.contains_key(&definition.initial_state) {
            return Err(JoyRouterError::ConfigError(
                format!("Initial state '{}' not found in state machine '{}'", 
                    definition.initial_state, definition.name)
            ));
        }
        
        let logger = Logger::new(&format!("state_machine_{}", definition.name));
        
        let state = StateMachineState {
            current_state: definition.initial_state.clone(),
            state_enter_time: Instant::now(),
            pending_transition: None,
            sequence_tracker: ButtonSequenceTracker {
                current_sequence: Vec::new(),
                sequence_start: None,
                expected_sequence: None,
                timeout_ms: 1000, // Default timeout
            },
            hold_tracker: ButtonHoldTracker {
                hold_starts: HashMap::new(),
            },
        };
        
        pr_info!(logger, "Created state machine '{}' starting in state '{}'", 
            definition.name, definition.initial_state);
        
        Ok(Self {
            definition,
            state,
            logger,
        })
    }
    
    /// Update the state machine with current button state
    pub fn update(&mut self, button_tracker: &ButtonTracker) -> JoyRouterResult<Vec<ActionType>> {
        let mut actions_to_execute = Vec::new();
        let now = Instant::now();
        
        // Update button hold tracking
        self.update_hold_tracking(button_tracker, now);
        
        // Update button sequence tracking
        self.update_sequence_tracking(button_tracker, now);
        
        // Check for pending transition
        if let Some(pending) = self.state.pending_transition.clone() {
            if now >= pending.trigger_time {
                actions_to_execute.extend(self.transition_to_state(&pending.target_state, now)?);
                self.state.pending_transition = None;
            }
        }
        
        // Check for timeout transitions
        let timeout_info = if let Some(current_state_def) = self.definition.states.get(&self.state.current_state) {
            if let (Some(timeout_ms), Some(ref timeout_target)) = 
                (current_state_def.timeout_ms, &current_state_def.timeout_target) {
                let timeout_duration = Duration::from_millis(timeout_ms);
                if now.duration_since(self.state.state_enter_time) >= timeout_duration {
                    Some((timeout_ms, timeout_target.clone()))
                } else {
                    None
                }
            } else {
                None
            }
        } else {
            None
        };
        
        if let Some((timeout_ms, timeout_target)) = timeout_info {
            pr_debug!(self.logger, "State '{}' timed out after {}ms, transitioning to '{}'", 
                self.state.current_state, timeout_ms, timeout_target);
            actions_to_execute.extend(self.transition_to_state(&timeout_target, now)?);
        }
        
        // Check transition conditions (only if no pending transition)
        if self.state.pending_transition.is_none() {
            if let Some(transition) = self.check_transitions(button_tracker, now)? {
                if transition.delay_ms > 0 {
                    // Schedule delayed transition
                    self.state.pending_transition = Some(PendingTransition {
                        target_state: transition.target_state.clone(),
                        trigger_time: now + Duration::from_millis(transition.delay_ms),
                    });
                    pr_debug!(self.logger, "Scheduled transition to '{}' in {}ms", 
                        transition.target_state, transition.delay_ms);
                } else {
                    // Immediate transition
                    actions_to_execute.extend(self.transition_to_state(&transition.target_state, now)?);
                }
            }
        }
        
        // Execute on_update actions for current state
        if let Some(current_state_def) = self.definition.states.get(&self.state.current_state) {
            actions_to_execute.extend(current_state_def.on_update.clone());
        }
        
        Ok(actions_to_execute)
    }
    
    /// Update button hold tracking
    fn update_hold_tracking(&mut self, button_tracker: &ButtonTracker, now: Instant) {
        // Track new button presses
        for button_idx in 0..16 { // Assume max 16 buttons
            if button_tracker.just_pressed(button_idx) {
                self.state.hold_tracker.hold_starts.insert(button_idx, now);
            } else if button_tracker.just_released(button_idx) {
                self.state.hold_tracker.hold_starts.remove(&button_idx);
            }
        }
    }
    
    /// Update button sequence tracking
    fn update_sequence_tracking(&mut self, button_tracker: &ButtonTracker, now: Instant) {
        // Check for new button presses to add to sequence
        for button_idx in 0..16 { // Assume max 16 buttons
            if button_tracker.just_pressed(button_idx) {
                if self.state.sequence_tracker.current_sequence.is_empty() {
                    self.state.sequence_tracker.sequence_start = Some(now);
                }
                self.state.sequence_tracker.current_sequence.push(button_idx);
                
                pr_debug!(self.logger, "Button sequence: {:?}", 
                    self.state.sequence_tracker.current_sequence);
            }
        }
        
        // Check for sequence timeout
        if let Some(start_time) = self.state.sequence_tracker.sequence_start {
            let timeout = Duration::from_millis(self.state.sequence_tracker.timeout_ms);
            if now.duration_since(start_time) >= timeout {
                pr_debug!(self.logger, "Button sequence timed out, clearing");
                self.state.sequence_tracker.current_sequence.clear();
                self.state.sequence_tracker.sequence_start = None;
            }
        }
    }
    
    /// Check all transition conditions and return the highest priority match
    fn check_transitions(&mut self, button_tracker: &ButtonTracker, now: Instant) -> JoyRouterResult<Option<StateTransition>> {
        let current_state_def = match self.definition.states.get(&self.state.current_state) {
            Some(state) => state,
            None => return Ok(None),
        };
        
        // Sort transitions by priority (highest first)
        let mut transitions = current_state_def.transitions.clone();
        transitions.sort_by(|a, b| b.priority.cmp(&a.priority));
        
        for transition in transitions {
            if self.check_transition_condition(&transition.condition, button_tracker, now)? {
                pr_debug!(self.logger, "Transition condition met: {:?}", transition.condition);
                return Ok(Some(transition));
            }
        }
        
        Ok(None)
    }
    
    /// Check if a specific transition condition is met
    fn check_transition_condition(
        &mut self, 
        condition: &TransitionCondition, 
        button_tracker: &ButtonTracker, 
        now: Instant
    ) -> JoyRouterResult<bool> {
        match condition {
            TransitionCondition::ButtonPress { button } => {
                Ok(button_tracker.just_pressed(*button))
            }
            TransitionCondition::ButtonRelease { button } => {
                Ok(button_tracker.just_released(*button))
            }
            TransitionCondition::ButtonHeld { button, duration_ms } => {
                if let Some(hold_start) = self.state.hold_tracker.hold_starts.get(button) {
                    let hold_duration = now.duration_since(*hold_start);
                    Ok(hold_duration >= Duration::from_millis(*duration_ms) && 
                       button_tracker.is_pressed(*button))
                } else {
                    Ok(false)
                }
            }
            TransitionCondition::ButtonCombo { buttons, require_all } => {
                if *require_all {
                    Ok(buttons.iter().all(|&btn| button_tracker.is_pressed(btn)))
                } else {
                    Ok(buttons.iter().any(|&btn| button_tracker.is_pressed(btn)))
                }
            }
            TransitionCondition::ButtonSequence { buttons, timeout_ms } => {
                // Update expected sequence for comparison
                self.state.sequence_tracker.expected_sequence = Some(buttons.clone());
                self.state.sequence_tracker.timeout_ms = *timeout_ms;
                
                // Check if current sequence matches expected
                Ok(self.state.sequence_tracker.current_sequence == *buttons)
            }
            TransitionCondition::Always => Ok(true),
            TransitionCondition::Custom { condition_id } => {
                pr_warn!(self.logger, "Custom condition '{}' not implemented", condition_id);
                Ok(false)
            }
        }
    }
    
    /// Transition to a new state
    fn transition_to_state(&mut self, target_state: &str, now: Instant) -> JoyRouterResult<Vec<ActionType>> {
        let mut actions = Vec::new();
        
        // Validate target state exists
        if !self.definition.states.contains_key(target_state) {
            return Err(JoyRouterError::ConfigError(
                format!("Target state '{}' not found in state machine '{}'", 
                    target_state, self.definition.name)
            ));
        }
        
        // Execute on_exit actions for current state
        if let Some(current_state_def) = self.definition.states.get(&self.state.current_state) {
            actions.extend(current_state_def.on_exit.clone());
        }
        
        pr_info!(self.logger, "Transitioning from '{}' to '{}'", 
            self.state.current_state, target_state);
        
        // Update state
        let _old_state = self.state.current_state.clone();
        self.state.current_state = target_state.to_string();
        self.state.state_enter_time = now;
        self.state.pending_transition = None;
        
        // Clear sequence tracking on state change
        self.state.sequence_tracker.current_sequence.clear();
        self.state.sequence_tracker.sequence_start = None;
        
        // Execute on_enter actions for new state
        if let Some(new_state_def) = self.definition.states.get(target_state) {
            actions.extend(new_state_def.on_enter.clone());
        }
        
        Ok(actions)
    }
    
    /// Get current state name
    pub fn current_state(&self) -> &str {
        &self.state.current_state
    }
    
    /// Get state machine name
    pub fn name(&self) -> &str {
        &self.definition.name
    }
    
    /// Force transition to a specific state (for external control)
    pub fn force_transition(&mut self, target_state: &str) -> JoyRouterResult<Vec<ActionType>> {
        let now = Instant::now();
        self.transition_to_state(target_state, now)
    }
    
    /// Reset state machine to initial state
    pub fn reset(&mut self) -> JoyRouterResult<Vec<ActionType>> {
        let initial_state = self.definition.initial_state.clone();
        self.force_transition(&initial_state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::button_tracker::ButtonTracker;
    
    fn create_test_state_machine() -> StateMachineDefinition {
        let mut states = HashMap::new();
        
        // Idle state
        states.insert("idle".to_string(), StateDefinition {
            name: "idle".to_string(),
            description: "Initial idle state".to_string(),
            on_enter: vec![],
            on_update: vec![],
            on_exit: vec![],
            transitions: vec![
                StateTransition {
                    target_state: "armed".to_string(),
                    condition: TransitionCondition::ButtonPress { button: 0 },
                    delay_ms: 0,
                    priority: 1,
                }
            ],
            timeout_ms: None,
            timeout_target: None,
        });
        
        // Armed state
        states.insert("armed".to_string(), StateDefinition {
            name: "armed".to_string(),
            description: "Armed and ready state".to_string(),
            on_enter: vec![ActionType::NoAction],
            on_update: vec![],
            on_exit: vec![],
            transitions: vec![
                StateTransition {
                    target_state: "firing".to_string(),
                    condition: TransitionCondition::ButtonPress { button: 1 },
                    delay_ms: 0,
                    priority: 1,
                },
                StateTransition {
                    target_state: "idle".to_string(),
                    condition: TransitionCondition::ButtonPress { button: 2 },
                    delay_ms: 0,
                    priority: 1,
                }
            ],
            timeout_ms: Some(5000),
            timeout_target: Some("idle".to_string()),
        });
        
        // Firing state
        states.insert("firing".to_string(), StateDefinition {
            name: "firing".to_string(),
            description: "Firing state".to_string(),
            on_enter: vec![ActionType::NoAction],
            on_update: vec![],
            on_exit: vec![],
            transitions: vec![
                StateTransition {
                    target_state: "idle".to_string(),
                    condition: TransitionCondition::Always,
                    delay_ms: 1000,
                    priority: 1,
                }
            ],
            timeout_ms: None,
            timeout_target: None,
        });
        
        StateMachineDefinition {
            name: "test_machine".to_string(),
            description: "Test state machine".to_string(),
            initial_state: "idle".to_string(),
            states,
        }
    }
    
    #[test]
    fn test_state_machine_creation() {
        let definition = create_test_state_machine();
        let state_machine = StateMachine::new(definition);
        assert!(state_machine.is_ok());
        
        let sm = state_machine.unwrap();
        assert_eq!(sm.current_state(), "idle");
    }
    
    #[test]
    fn test_state_machine_invalid_initial() {
        let mut definition = create_test_state_machine();
        definition.initial_state = "nonexistent".to_string();
        
        let result = StateMachine::new(definition);
        assert!(result.is_err());
    }
    
    #[test]
    fn test_basic_transition() {
        let definition = create_test_state_machine();
        let mut state_machine = StateMachine::new(definition).unwrap();
        let mut button_tracker = ButtonTracker::new();
        
        // Simulate button 0 press
        button_tracker.update(&[1, 0, 0]);  // Button 0 pressed
        
        let actions = state_machine.update(&button_tracker).unwrap();
        assert_eq!(state_machine.current_state(), "armed");
        assert!(!actions.is_empty()); // Should have on_enter actions
    }
    
    #[test]
    fn test_button_sequence_condition() {
        let condition = TransitionCondition::ButtonSequence {
            buttons: vec![0, 1, 2],
            timeout_ms: 1000,
        };
        
        match condition {
            TransitionCondition::ButtonSequence { buttons, timeout_ms } => {
                assert_eq!(buttons, vec![0, 1, 2]);
                assert_eq!(timeout_ms, 1000);
            }
            _ => panic!("Expected ButtonSequence condition"),
        }
    }
    
    #[test]
    fn test_button_combo_condition() {
        let condition = TransitionCondition::ButtonCombo {
            buttons: vec![0, 1],
            require_all: true,
        };
        
        match condition {
            TransitionCondition::ButtonCombo { buttons, require_all } => {
                assert_eq!(buttons, vec![0, 1]);
                assert!(require_all);
            }
            _ => panic!("Expected ButtonCombo condition"),
        }
    }
}