use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use crate::state_machine::{StateMachine, StateMachineDefinition};
use crate::button_tracker::ButtonTracker;
use crate::config::{ActionType, StateMachineActionType};
use crate::error::{JoyRouterError, JoyRouterResult};
use crate::command_queue::{CommandQueue, Command, Priority, PrioritizedCommand};
use safe_drive::logger::Logger;
use safe_drive::{pr_debug, pr_info, pr_warn};

/// Manages multiple state machines
pub struct StateMachineManager {
    /// Running state machines
    state_machines: Arc<Mutex<HashMap<String, StateMachine>>>,
    /// State machine definitions
    definitions: HashMap<String, StateMachineDefinition>,
    /// Command queue for executing actions
    command_queue: Arc<CommandQueue>,
    /// Logger
    logger: Logger,
}

impl StateMachineManager {
    /// Create a new state machine manager
    pub fn new(
        definitions: HashMap<String, StateMachineDefinition>,
        command_queue: Arc<CommandQueue>
    ) -> Self {
        let logger = Logger::new("state_machine_manager");
        
        pr_info!(logger, "Created state machine manager with {} definitions", definitions.len());
        
        Self {
            state_machines: Arc::new(Mutex::new(HashMap::new())),
            definitions,
            command_queue,
            logger,
        }
    }
    
    /// Start a state machine by name
    pub fn start_state_machine(&self, name: &str) -> JoyRouterResult<()> {
        let definition = self.definitions.get(name)
            .ok_or_else(|| JoyRouterError::ConfigError(
                format!("State machine '{}' not found", name)
            ))?;
        
        let state_machine = StateMachine::new(definition.clone())?;
        
        let mut machines = self.state_machines.lock()
            .map_err(|e| JoyRouterError::CommandError(
                format!("Failed to lock state machines: {}", e)
            ))?;
        
        machines.insert(name.to_string(), state_machine);
        
        pr_info!(self.logger, "Started state machine '{}'", name);
        Ok(())
    }
    
    /// Stop a state machine by name
    pub fn stop_state_machine(&self, name: &str) -> JoyRouterResult<()> {
        let mut machines = self.state_machines.lock()
            .map_err(|e| JoyRouterError::CommandError(
                format!("Failed to lock state machines: {}", e)
            ))?;
        
        if machines.remove(name).is_some() {
            pr_info!(self.logger, "Stopped state machine '{}'", name);
        } else {
            pr_warn!(self.logger, "Attempted to stop non-running state machine '{}'", name);
        }
        
        Ok(())
    }
    
    /// Update all running state machines
    pub fn update(&self, button_tracker: &ButtonTracker) -> JoyRouterResult<()> {
        let mut machines = self.state_machines.lock()
            .map_err(|e| JoyRouterError::CommandError(
                format!("Failed to lock state machines: {}", e)
            ))?;
        
        let mut actions_to_execute = Vec::new();
        
        // Update each running state machine
        for (name, state_machine) in machines.iter_mut() {
            match state_machine.update(button_tracker) {
                Ok(actions) => {
                    if !actions.is_empty() {
                        pr_debug!(self.logger, "State machine '{}' generated {} actions", name, actions.len());
                        actions_to_execute.extend(actions);
                    }
                }
                Err(e) => {
                    pr_warn!(self.logger, "Error updating state machine '{}': {:?}", name, e);
                }
            }
        }
        
        // Execute all actions
        for action in actions_to_execute {
            self.execute_action(action)?;
        }
        
        Ok(())
    }
    
    /// Execute a state machine action
    pub fn execute_state_machine_action(
        &self, 
        state_machine_name: &str, 
        action: &StateMachineActionType
    ) -> JoyRouterResult<()> {
        let mut machines = self.state_machines.lock()
            .map_err(|e| JoyRouterError::CommandError(
                format!("Failed to lock state machines: {}", e)
            ))?;
        
        match action {
            StateMachineActionType::Start => {
                drop(machines); // Release lock before calling start_state_machine
                self.start_state_machine(state_machine_name)
            }
            StateMachineActionType::Stop => {
                drop(machines); // Release lock before calling stop_state_machine
                self.stop_state_machine(state_machine_name)
            }
            StateMachineActionType::Reset => {
                if let Some(state_machine) = machines.get_mut(state_machine_name) {
                    let actions = state_machine.reset()?;
                    drop(machines); // Release lock before executing actions
                    for action in actions {
                        self.execute_action(action)?;
                    }
                    pr_info!(self.logger, "Reset state machine '{}'", state_machine_name);
                } else {
                    return Err(JoyRouterError::ConfigError(
                        format!("State machine '{}' is not running", state_machine_name)
                    ));
                }
                Ok(())
            }
            StateMachineActionType::TransitionTo { state } => {
                if let Some(state_machine) = machines.get_mut(state_machine_name) {
                    let actions = state_machine.force_transition(state)?;
                    drop(machines); // Release lock before executing actions
                    for action in actions {
                        self.execute_action(action)?;
                    }
                    pr_info!(self.logger, "Forced transition of '{}' to state '{}'", state_machine_name, state);
                } else {
                    return Err(JoyRouterError::ConfigError(
                        format!("State machine '{}' is not running", state_machine_name)
                    ));
                }
                Ok(())
            }
            StateMachineActionType::TriggerCondition { condition_id } => {
                pr_warn!(self.logger, "TriggerCondition action not implemented for condition '{}'", condition_id);
                Ok(())
            }
        }
    }
    
    /// Convert ActionType to Command and enqueue it
    fn execute_action(&self, action: ActionType) -> JoyRouterResult<()> {
        let command = self.action_to_command(action)?;
        let sender = self.command_queue.get_sender();
        
        sender.send(PrioritizedCommand {
            command,
            priority: Priority::Normal,
        }).map_err(|e| JoyRouterError::CommandError(
            format!("Failed to enqueue state machine action: {}", e)
        ))?;
        
        Ok(())
    }
    
    /// Convert ActionType to Command (similar to sequence_executor)
    fn action_to_command(&self, action: ActionType) -> JoyRouterResult<Command> {
        match action {
            ActionType::PublishTwist { linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, .. } => {
                let mut twist = geometry_msgs::msg::Twist::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create Twist".to_string()))?;
                twist.linear.x = linear_x;
                twist.linear.y = linear_y;
                twist.linear.z = linear_z;
                twist.angular.x = angular_x;
                twist.angular.y = angular_y;
                twist.angular.z = angular_z;
                Ok(Command::PublishTwist(twist))
            }
            ActionType::PublishTwistStamped { linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, frame_id, .. } => {
                let mut twist_stamped = geometry_msgs::msg::TwistStamped::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create TwistStamped".to_string()))?;
                
                if let Some(frame_id_ros) = safe_drive::msg::RosString::<0>::new(&frame_id) {
                    twist_stamped.header.frame_id = frame_id_ros;
                }
                twist_stamped.twist.linear.x = linear_x;
                twist_stamped.twist.linear.y = linear_y;
                twist_stamped.twist.linear.z = linear_z;
                twist_stamped.twist.angular.x = angular_x;
                twist_stamped.twist.angular.y = angular_y;
                twist_stamped.twist.angular.z = angular_z;
                Ok(Command::PublishTwistStamped(twist_stamped))
            }
            ActionType::PublishBool { topic, value, .. } => {
                let mut bool_msg = std_msgs::msg::Bool::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create Bool".to_string()))?;
                bool_msg.data = value;
                Ok(Command::PublishBool { topic, value: bool_msg })
            }
            ActionType::PublishInt32 { topic, value, .. } => {
                let mut int_msg = std_msgs::msg::Int32::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create Int32".to_string()))?;
                int_msg.data = value;
                Ok(Command::PublishInt32 { topic, value: int_msg })
            }
            ActionType::PublishFloat64 { topic, value, .. } => {
                let mut float_msg = std_msgs::msg::Float64::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create Float64".to_string()))?;
                float_msg.data = value;
                Ok(Command::PublishFloat64 { topic, value: float_msg })
            }
            ActionType::PublishString { topic, value, .. } => {
                let mut str_msg = std_msgs::msg::String::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create String".to_string()))?;
                if let Some(data_ros) = safe_drive::msg::RosString::<0>::new(&value) {
                    str_msg.data = data_ros;
                }
                Ok(Command::PublishString { topic, value: str_msg })
            }
            ActionType::CallService { service_name, service_type, .. } => {
                Ok(Command::CallService { service_name, service_type })
            }
            ActionType::NoAction => {
                Ok(Command::Stop)
            }
            ActionType::ActionSequence { .. } => {
                Err(JoyRouterError::ConfigError(
                    "Action sequences within state machines not supported".to_string()
                ))
            }
            ActionType::ExecuteMacro { .. } => {
                Err(JoyRouterError::ConfigError(
                    "Macro execution within state machines not supported".to_string()
                ))
            }
            ActionType::StateMachineAction { state_machine, action } => {
                // Handle state machine actions by delegating to the manager
                // This allows state machines to control other state machines
                pr_debug!(self.logger, "State machine action: {} -> {:?}", state_machine, action);
                Ok(Command::Stop) // Placeholder - this should be handled differently
            }
        }
    }
    
    /// Get status of all running state machines
    pub fn get_status(&self) -> JoyRouterResult<Vec<(String, String)>> {
        let machines = self.state_machines.lock()
            .map_err(|e| JoyRouterError::CommandError(
                format!("Failed to lock state machines: {}", e)
            ))?;
        
        let mut status = Vec::new();
        for (name, state_machine) in machines.iter() {
            status.push((name.clone(), state_machine.current_state().to_string()));
        }
        
        Ok(status)
    }
    
    /// Stop all running state machines
    pub fn stop_all(&self) -> JoyRouterResult<()> {
        let mut machines = self.state_machines.lock()
            .map_err(|e| JoyRouterError::CommandError(
                format!("Failed to lock state machines: {}", e)
            ))?;
        
        let count = machines.len();
        machines.clear();
        
        pr_info!(self.logger, "Stopped {} state machines", count);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command_queue::CommandQueue;
    use crate::state_machine::{StateDefinition, StateTransition, TransitionCondition};
    
    fn create_test_definitions() -> HashMap<String, StateMachineDefinition> {
        let mut definitions = HashMap::new();
        
        let mut states = HashMap::new();
        states.insert("idle".to_string(), StateDefinition {
            name: "idle".to_string(),
            description: "Initial state".to_string(),
            on_enter: vec![],
            on_update: vec![],
            on_exit: vec![],
            transitions: vec![
                StateTransition {
                    target_state: "active".to_string(),
                    condition: TransitionCondition::ButtonPress { button: 0 },
                    delay_ms: 0,
                    priority: 1,
                }
            ],
            timeout_ms: None,
            timeout_target: None,
        });
        
        states.insert("active".to_string(), StateDefinition {
            name: "active".to_string(),
            description: "Active state".to_string(),
            on_enter: vec![ActionType::NoAction],
            on_update: vec![],
            on_exit: vec![],
            transitions: vec![],
            timeout_ms: None,
            timeout_target: None,
        });
        
        definitions.insert("test_machine".to_string(), StateMachineDefinition {
            name: "test_machine".to_string(),
            description: "Test machine".to_string(),
            initial_state: "idle".to_string(),
            states,
        });
        
        definitions
    }
    
    #[test]
    fn test_state_machine_manager_creation() {
        let definitions = create_test_definitions();
        let queue = Arc::new(CommandQueue::new());
        let manager = StateMachineManager::new(definitions, queue);
        
        let status = manager.get_status().unwrap();
        assert_eq!(status.len(), 0); // No machines running initially
    }
    
    #[test]
    fn test_start_stop_state_machine() {
        let definitions = create_test_definitions();
        let queue = Arc::new(CommandQueue::new());
        let manager = StateMachineManager::new(definitions, queue);
        
        // Start state machine
        manager.start_state_machine("test_machine").unwrap();
        
        let status = manager.get_status().unwrap();
        assert_eq!(status.len(), 1);
        assert_eq!(status[0].0, "test_machine");
        assert_eq!(status[0].1, "idle");
        
        // Stop state machine
        manager.stop_state_machine("test_machine").unwrap();
        
        let status = manager.get_status().unwrap();
        assert_eq!(status.len(), 0);
    }
    
    #[test]
    fn test_start_nonexistent_machine() {
        let definitions = create_test_definitions();
        let queue = Arc::new(CommandQueue::new());
        let manager = StateMachineManager::new(definitions, queue);
        
        let result = manager.start_state_machine("nonexistent");
        assert!(result.is_err());
    }
}