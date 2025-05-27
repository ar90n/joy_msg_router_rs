use std::collections::HashMap;

/// Tracks button states and detects press/release events
#[derive(Debug, Clone)]
pub struct ButtonTracker {
    /// Previous frame button states
    previous_states: HashMap<usize, bool>,
    /// Current frame button states
    current_states: HashMap<usize, bool>,
}

impl ButtonTracker {
    /// Create a new button tracker
    pub fn new() -> Self {
        Self {
            previous_states: HashMap::new(),
            current_states: HashMap::new(),
        }
    }
    
    /// Update button states from Joy message buttons
    pub fn update(&mut self, buttons: &[i32]) {
        // Move current to previous
        self.previous_states = self.current_states.clone();
        
        // Clear current states
        self.current_states.clear();
        
        // Update current states from button array
        for (index, &value) in buttons.iter().enumerate() {
            self.current_states.insert(index, value != 0);
        }
    }
    
    /// Check if a button is currently pressed
    pub fn is_pressed(&self, button_index: usize) -> bool {
        self.current_states.get(&button_index).copied().unwrap_or(false)
    }
    
    /// Check if a button was just pressed (rising edge)
    pub fn just_pressed(&self, button_index: usize) -> bool {
        let was_pressed = self.previous_states.get(&button_index).copied().unwrap_or(false);
        let is_pressed = self.current_states.get(&button_index).copied().unwrap_or(false);
        !was_pressed && is_pressed
    }
    
    /// Check if a button was just released (falling edge)
    pub fn just_released(&self, button_index: usize) -> bool {
        let was_pressed = self.previous_states.get(&button_index).copied().unwrap_or(false);
        let is_pressed = self.current_states.get(&button_index).copied().unwrap_or(false);
        was_pressed && !is_pressed
    }
    
    /// Check if a button is being held (was pressed and still is)
    pub fn is_held(&self, button_index: usize) -> bool {
        let was_pressed = self.previous_states.get(&button_index).copied().unwrap_or(false);
        let is_pressed = self.current_states.get(&button_index).copied().unwrap_or(false);
        was_pressed && is_pressed
    }
    
    /// Get all currently pressed button indices
    pub fn get_pressed_buttons(&self) -> Vec<usize> {
        self.current_states
            .iter()
            .filter_map(|(&index, &pressed)| if pressed { Some(index) } else { None })
            .collect()
    }
    
    /// Get all buttons that were just pressed
    pub fn get_just_pressed_buttons(&self) -> Vec<usize> {
        self.current_states
            .iter()
            .filter_map(|(&index, &pressed)| {
                if pressed && !self.previous_states.get(&index).copied().unwrap_or(false) {
                    Some(index)
                } else {
                    None
                }
            })
            .collect()
    }
    
    /// Check if multiple buttons are pressed simultaneously
    pub fn are_all_pressed(&self, button_indices: &[usize]) -> bool {
        button_indices.iter().all(|&index| self.is_pressed(index))
    }
    
    /// Check if any of the specified buttons are pressed
    pub fn is_any_pressed(&self, button_indices: &[usize]) -> bool {
        button_indices.iter().any(|&index| self.is_pressed(index))
    }
}

impl Default for ButtonTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_button_press_detection() {
        let mut tracker = ButtonTracker::new();
        
        // Initial state - no buttons pressed
        tracker.update(&[0, 0, 0]);
        assert!(!tracker.is_pressed(0));
        assert!(!tracker.just_pressed(0));
        assert!(!tracker.just_released(0));
        
        // Press button 0
        tracker.update(&[1, 0, 0]);
        assert!(tracker.is_pressed(0));
        assert!(tracker.just_pressed(0));
        assert!(!tracker.just_released(0));
        assert!(!tracker.is_held(0));
        
        // Hold button 0
        tracker.update(&[1, 0, 0]);
        assert!(tracker.is_pressed(0));
        assert!(!tracker.just_pressed(0));
        assert!(!tracker.just_released(0));
        assert!(tracker.is_held(0));
        
        // Release button 0
        tracker.update(&[0, 0, 0]);
        assert!(!tracker.is_pressed(0));
        assert!(!tracker.just_pressed(0));
        assert!(tracker.just_released(0));
        assert!(!tracker.is_held(0));
    }
    
    #[test]
    fn test_multiple_buttons() {
        let mut tracker = ButtonTracker::new();
        
        // Press multiple buttons
        tracker.update(&[1, 0, 1, 0, 1]);
        let mut pressed = tracker.get_pressed_buttons();
        pressed.sort();
        assert_eq!(pressed, vec![0, 2, 4]);
        
        let mut just_pressed = tracker.get_just_pressed_buttons();
        just_pressed.sort();
        assert_eq!(just_pressed, vec![0, 2, 4]);
        
        // Hold some, release others, press new ones
        tracker.update(&[1, 1, 0, 0, 1]);
        let mut pressed = tracker.get_pressed_buttons();
        pressed.sort();
        assert_eq!(pressed, vec![0, 1, 4]);
        
        let mut just_pressed = tracker.get_just_pressed_buttons();
        just_pressed.sort();
        assert_eq!(just_pressed, vec![1]);
        
        assert!(tracker.just_released(2));
    }
    
    #[test]
    fn test_button_combinations() {
        let mut tracker = ButtonTracker::new();
        
        tracker.update(&[1, 1, 0, 1]);
        assert!(tracker.are_all_pressed(&[0, 1, 3]));
        assert!(!tracker.are_all_pressed(&[0, 1, 2, 3]));
        assert!(tracker.is_any_pressed(&[0, 2]));
        assert!(!tracker.is_any_pressed(&[2, 5]));
    }
    
    #[test]
    fn test_expanding_button_array() {
        let mut tracker = ButtonTracker::new();
        
        // Start with 3 buttons
        tracker.update(&[1, 0, 1]);
        assert!(tracker.is_pressed(0));
        assert!(tracker.is_pressed(2));
        
        // Expand to 5 buttons
        tracker.update(&[1, 0, 1, 0, 1]);
        assert!(tracker.is_pressed(0));
        assert!(tracker.is_pressed(2));
        assert!(tracker.just_pressed(4));
        
        // Shrink back to 2 buttons
        tracker.update(&[0, 1]);
        assert!(!tracker.is_pressed(0));
        assert!(tracker.is_pressed(1));
        assert!(!tracker.is_pressed(2)); // No longer exists
    }
}