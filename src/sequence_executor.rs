use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std::collections::HashMap;
use crate::config::{SequenceStep, ActionType, MacroDefinition, MacroParameter};
use crate::command_queue::{CommandQueue, Command, Priority, PrioritizedCommand};
use crate::error::{JoyRouterError, JoyRouterResult};
use safe_drive::logger::Logger;
use safe_drive::{pr_debug, pr_info, pr_warn};
use geometry_msgs::msg::{Twist, TwistStamped};
use std_msgs::msg::{Bool, Int32, Float64, String as StringMsg};

/// State of a running sequence
#[derive(Debug, Clone)]
pub struct SequenceState {
    /// Current step index
    pub current_step: usize,
    /// When the sequence started
    pub start_time: Instant,
    /// When the current step started
    pub step_start_time: Instant,
    /// Whether the sequence should repeat
    pub repeat: bool,
    /// Whether the sequence is active
    pub active: bool,
    /// The sequence steps
    pub steps: Vec<SequenceStep>,
    /// Unique ID for this sequence execution
    pub id: String,
}

/// Manages execution of action sequences and macros
pub struct SequenceExecutor {
    /// Currently running sequences
    running_sequences: Arc<Mutex<HashMap<String, SequenceState>>>,
    /// Command queue for sending commands
    command_queue: Arc<CommandQueue>,
    /// Logger for sequence execution
    logger: Logger,
}

impl SequenceExecutor {
    /// Create a new sequence executor
    pub fn new(command_queue: Arc<CommandQueue>) -> Self {
        Self {
            running_sequences: Arc::new(Mutex::new(HashMap::new())),
            command_queue,
            logger: Logger::new("sequence_executor"),
        }
    }
    
    /// Start executing a sequence
    pub fn start_sequence(&self, steps: Vec<SequenceStep>, repeat: bool, sequence_id: String) -> JoyRouterResult<()> {
        if steps.is_empty() {
            return Err(JoyRouterError::ConfigError(
                "Cannot start empty sequence".to_string()
            ));
        }
        
        let step_count = steps.len();
        let now = Instant::now();
        let state = SequenceState {
            current_step: 0,
            start_time: now,
            step_start_time: now,
            repeat,
            active: true,
            steps,
            id: sequence_id.clone(),
        };
        
        // Store the sequence state
        let mut sequences = self.running_sequences.lock()
            .map_err(|e| JoyRouterError::CommandError(
                format!("Failed to lock sequence state: {}", e)
            ))?;
        sequences.insert(sequence_id.clone(), state);
        
        pr_info!(self.logger, "Started sequence '{}' with {} steps", sequence_id, step_count);
        
        Ok(())
    }
    
    /// Stop a running sequence
    pub fn stop_sequence(&self, sequence_id: &str) -> JoyRouterResult<()> {
        let mut sequences = self.running_sequences.lock()
            .map_err(|e| JoyRouterError::CommandError(
                format!("Failed to lock sequence state: {}", e)
            ))?;
        
        if let Some(state) = sequences.get_mut(sequence_id) {
            state.active = false;
            pr_info!(self.logger, "Stopped sequence '{}'", sequence_id);
        }
        
        Ok(())
    }
    
    /// Process all running sequences and execute next steps as needed
    pub fn process_sequences(&self) -> JoyRouterResult<()> {
        let mut sequences = self.running_sequences.lock()
            .map_err(|e| JoyRouterError::CommandError(
                format!("Failed to lock sequence state: {}", e)
            ))?;
        
        let now = Instant::now();
        let mut to_remove = Vec::new();
        
        for (id, state) in sequences.iter_mut() {
            if !state.active {
                to_remove.push(id.clone());
                continue;
            }
            
            // Check if it's time to execute the next step
            if let Some(current_step) = state.steps.get(state.current_step) {
                let step_elapsed = now.duration_since(state.step_start_time);
                let delay = Duration::from_millis(current_step.delay_ms);
                
                if step_elapsed >= delay {
                    // Execute the current step
                    if let Err(e) = self.execute_step(current_step) {
                        pr_warn!(self.logger, "Failed to execute sequence step: {:?}", e);
                    }
                    
                    // Check if step has a duration and if it's time to move to next step
                    let duration = Duration::from_millis(current_step.duration_ms);
                    let should_advance = if current_step.duration_ms == 0 {
                        // Instantaneous action, advance immediately
                        true
                    } else {
                        // Check if duration has elapsed
                        step_elapsed >= delay + duration
                    };
                    
                    if should_advance {
                        state.current_step += 1;
                        state.step_start_time = now;
                        
                        // Check if sequence is complete
                        if state.current_step >= state.steps.len() {
                            if state.repeat {
                                // Restart the sequence
                                state.current_step = 0;
                                pr_debug!(self.logger, "Restarting sequence '{}'", id);
                            } else {
                                // Mark sequence as complete
                                state.active = false;
                                pr_info!(self.logger, "Completed sequence '{}'", id);
                            }
                        }
                    }
                }
            } else {
                // Invalid step index, deactivate sequence
                state.active = false;
                pr_warn!(self.logger, "Invalid step index in sequence '{}', deactivating", id);
            }
        }
        
        // Remove completed sequences
        for id in to_remove {
            sequences.remove(&id);
        }
        
        Ok(())
    }
    
    /// Execute a single sequence step
    fn execute_step(&self, step: &SequenceStep) -> JoyRouterResult<()> {
        let command = self.action_to_command(&step.action)?;
        
        // Send the command
        let sender = self.command_queue.get_sender();
        sender.send(PrioritizedCommand {
            command,
            priority: Priority::Normal,
        }).map_err(|e| JoyRouterError::CommandError(
            format!("Failed to send sequence command: {}", e)
        ))?;
        
        Ok(())
    }
    
    /// Convert an ActionType to a Command
    fn action_to_command(&self, action: &ActionType) -> JoyRouterResult<Command> {
        match action {
            ActionType::PublishTwist { linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, .. } => {
                let mut twist = Twist::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create Twist".to_string()))?;
                twist.linear.x = *linear_x;
                twist.linear.y = *linear_y;
                twist.linear.z = *linear_z;
                twist.angular.x = *angular_x;
                twist.angular.y = *angular_y;
                twist.angular.z = *angular_z;
                Ok(Command::PublishTwist(twist))
            }
            ActionType::PublishTwistStamped { linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, frame_id, .. } => {
                let mut twist_stamped = TwistStamped::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create TwistStamped".to_string()))?;
                
                // Create frame_id RosString
                if let Some(frame_id_ros) = safe_drive::msg::RosString::<0>::new(frame_id) {
                    twist_stamped.header.frame_id = frame_id_ros;
                }
                twist_stamped.twist.linear.x = *linear_x;
                twist_stamped.twist.linear.y = *linear_y;
                twist_stamped.twist.linear.z = *linear_z;
                twist_stamped.twist.angular.x = *angular_x;
                twist_stamped.twist.angular.y = *angular_y;
                twist_stamped.twist.angular.z = *angular_z;
                Ok(Command::PublishTwistStamped(twist_stamped))
            }
            ActionType::PublishBool { topic, value, .. } => {
                let mut bool_msg = Bool::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create Bool".to_string()))?;
                bool_msg.data = *value;
                Ok(Command::PublishBool { topic: topic.clone(), value: bool_msg })
            }
            ActionType::PublishInt32 { topic, value, .. } => {
                let mut int_msg = Int32::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create Int32".to_string()))?;
                int_msg.data = *value;
                Ok(Command::PublishInt32 { topic: topic.clone(), value: int_msg })
            }
            ActionType::PublishFloat64 { topic, value, .. } => {
                let mut float_msg = Float64::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create Float64".to_string()))?;
                float_msg.data = *value;
                Ok(Command::PublishFloat64 { topic: topic.clone(), value: float_msg })
            }
            ActionType::PublishString { topic, value, .. } => {
                let mut str_msg = StringMsg::new()
                    .ok_or_else(|| JoyRouterError::MessageError("Failed to create String".to_string()))?;
                if let Some(data_ros) = safe_drive::msg::RosString::<0>::new(value) {
                    str_msg.data = data_ros;
                }
                Ok(Command::PublishString { topic: topic.clone(), value: str_msg })
            }
            ActionType::CallService { service_name, service_type, .. } => {
                Ok(Command::CallService {
                    service_name: service_name.clone(),
                    service_type: service_type.clone(),
                })
            }
            ActionType::NoAction => {
                Ok(Command::Stop)
            }
            ActionType::ActionSequence { .. } => {
                Err(JoyRouterError::ConfigError(
                    "Nested sequences are not supported".to_string()
                ))
            }
            ActionType::ExecuteMacro { .. } => {
                Err(JoyRouterError::ConfigError(
                    "Macro execution within sequences not yet supported".to_string()
                ))
            }
            ActionType::StateMachineAction { .. } => {
                Err(JoyRouterError::ConfigError(
                    "State machine actions within sequences not supported".to_string()
                ))
            }
        }
    }
    
    /// Execute a macro with parameters
    pub fn execute_macro(&self, macro_def: &MacroDefinition, parameters: &HashMap<String, MacroParameter>) -> JoyRouterResult<()> {
        // Validate parameters
        for (param_name, param_def) in &macro_def.parameters {
            if !parameters.contains_key(param_name) {
                if param_def.default.is_none() {
                    return Err(JoyRouterError::ConfigError(
                        format!("Missing required parameter '{}' for macro '{}'", param_name, macro_def.name)
                    ));
                }
            }
        }
        
        // Create a sequence from the macro
        let sequence_id = format!("macro_{}_{}", macro_def.name, Instant::now().elapsed().as_millis());
        self.start_sequence(macro_def.actions.clone(), false, sequence_id)?;
        
        pr_info!(self.logger, "Started macro '{}' execution", macro_def.name);
        
        Ok(())
    }
    
    /// Get status of all running sequences
    pub fn get_sequence_status(&self) -> JoyRouterResult<Vec<(String, usize, usize)>> {
        let sequences = self.running_sequences.lock()
            .map_err(|e| JoyRouterError::CommandError(
                format!("Failed to lock sequence state: {}", e)
            ))?;
        
        let mut status = Vec::new();
        for (id, state) in sequences.iter() {
            if state.active {
                status.push((id.clone(), state.current_step, state.steps.len()));
            }
        }
        
        Ok(status)
    }
    
    /// Stop all running sequences
    pub fn stop_all_sequences(&self) -> JoyRouterResult<()> {
        let mut sequences = self.running_sequences.lock()
            .map_err(|e| JoyRouterError::CommandError(
                format!("Failed to lock sequence state: {}", e)
            ))?;
        
        for state in sequences.values_mut() {
            state.active = false;
        }
        
        pr_info!(self.logger, "Stopped all running sequences");
        
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command_queue::CommandQueue;
    
    #[test]
    fn test_sequence_executor_creation() {
        let queue = Arc::new(CommandQueue::new());
        let executor = SequenceExecutor::new(queue);
        
        // Should be able to create executor
        assert!(executor.get_sequence_status().unwrap().is_empty());
    }
    
    #[test]
    fn test_start_empty_sequence() {
        let queue = Arc::new(CommandQueue::new());
        let executor = SequenceExecutor::new(queue);
        
        let result = executor.start_sequence(vec![], false, "empty".to_string());
        assert!(result.is_err());
    }
    
    #[test]
    fn test_sequence_state_management() {
        let queue = Arc::new(CommandQueue::new());
        let executor = SequenceExecutor::new(queue);
        
        let steps = vec![
            SequenceStep {
                action: ActionType::NoAction,
                delay_ms: 0,
                duration_ms: 0,
            }
        ];
        
        executor.start_sequence(steps, false, "test".to_string()).unwrap();
        
        let status = executor.get_sequence_status().unwrap();
        assert_eq!(status.len(), 1);
        assert_eq!(status[0].0, "test");
        assert_eq!(status[0].1, 0); // current step
        assert_eq!(status[0].2, 1); // total steps
    }
}