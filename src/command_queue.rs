use std::sync::mpsc::{channel, Sender, Receiver};
use geometry_msgs::msg::Twist;
use safe_drive::error::DynError;

/// Represents different types of commands that can be queued
#[derive(Debug)]
pub enum Command {
    /// Publish a Twist message
    PublishTwist(Twist),
    /// Call a service (placeholder for now)
    CallService {
        service_name: String,
        service_type: String,
    },
    /// Stop all movement
    Stop,
}

/// Priority levels for commands
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
    Low = 0,
    Normal = 1,
    High = 2,
    Critical = 3,
}

/// A command with priority
#[derive(Debug)]
pub struct PrioritizedCommand {
    pub command: Command,
    pub priority: Priority,
}

/// Thread-safe command queue for decoupled command processing
pub struct CommandQueue {
    sender: Sender<PrioritizedCommand>,
    receiver: Receiver<PrioritizedCommand>,
}

impl CommandQueue {
    /// Creates a new command queue
    pub fn new() -> Self {
        let (sender, receiver) = channel();
        Self { sender, receiver }
    }

    /// Get a sender that can be cloned and used from multiple threads
    pub fn get_sender(&self) -> Sender<PrioritizedCommand> {
        self.sender.clone()
    }

    /// Enqueue a command with normal priority
    pub fn enqueue(&self, command: Command) -> Result<(), DynError> {
        self.enqueue_with_priority(command, Priority::Normal)
    }

    /// Enqueue a command with specific priority
    pub fn enqueue_with_priority(&self, command: Command, priority: Priority) -> Result<(), DynError> {
        self.sender
            .send(PrioritizedCommand { command, priority })
            .map_err(|e| format!("Failed to enqueue command: {}", e).into())
    }

    /// Try to dequeue a command (non-blocking)
    pub fn try_dequeue(&self) -> Option<PrioritizedCommand> {
        self.receiver.try_recv().ok()
    }

    /// Process all pending commands with a handler function
    pub fn process_pending<F>(&self, mut handler: F) -> Result<(), DynError>
    where
        F: FnMut(PrioritizedCommand) -> Result<(), DynError>,
    {
        // Collect all pending commands and sort by priority
        let mut commands = Vec::new();
        while let Some(cmd) = self.try_dequeue() {
            commands.push(cmd);
        }
        
        // Sort by priority (highest first)
        commands.sort_by(|a, b| b.priority.cmp(&a.priority));
        
        // Process commands in priority order
        for cmd in commands {
            handler(cmd)?;
        }
        
        Ok(())
    }
}

impl Default for CommandQueue {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_queue_basic() {
        let queue = CommandQueue::new();
        
        // Enqueue a command
        queue.enqueue(Command::Stop).unwrap();
        
        // Dequeue and verify
        let cmd = queue.try_dequeue().expect("Should have a command");
        matches!(cmd.command, Command::Stop);
        assert_eq!(cmd.priority, Priority::Normal);
    }

    #[test]
    fn test_command_queue_priority() {
        let queue = CommandQueue::new();
        
        // Enqueue commands with different priorities
        queue.enqueue_with_priority(Command::Stop, Priority::Low).unwrap();
        queue.enqueue_with_priority(Command::Stop, Priority::Critical).unwrap();
        queue.enqueue_with_priority(Command::Stop, Priority::Normal).unwrap();
        
        // Process and verify priority order
        let mut priorities = Vec::new();
        queue.process_pending(|cmd| {
            priorities.push(cmd.priority);
            Ok(())
        }).unwrap();
        
        assert_eq!(priorities, vec![Priority::Critical, Priority::Normal, Priority::Low]);
    }

    #[test]
    fn test_command_queue_multiple_senders() {
        let queue = CommandQueue::new();
        let sender1 = queue.get_sender();
        let sender2 = queue.get_sender();
        
        // Send from different senders
        sender1.send(PrioritizedCommand {
            command: Command::Stop,
            priority: Priority::Normal,
        }).unwrap();
        
        sender2.send(PrioritizedCommand {
            command: Command::Stop,
            priority: Priority::High,
        }).unwrap();
        
        // Verify both commands are received
        assert!(queue.try_dequeue().is_some());
        assert!(queue.try_dequeue().is_some());
        assert!(queue.try_dequeue().is_none());
    }
}