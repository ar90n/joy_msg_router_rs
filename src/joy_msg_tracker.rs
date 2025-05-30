use sensor_msgs::msg::Joy;
use std::collections::HashMap;

/// Tracks joy message states (axes and buttons) and detects press/release events
#[derive(Debug, Clone)]
pub struct JoyMsgTracker {
    previous_button_states: HashMap<usize, bool>,
    current_button_states: HashMap<usize, bool>,
    current_axes: Vec<f32>,
    previous_axes: Vec<f32>,
}

#[allow(dead_code)]
impl JoyMsgTracker {
    pub fn new() -> Self {
        Self {
            previous_button_states: HashMap::new(),
            current_button_states: HashMap::new(),
            current_axes: Vec::new(),
            previous_axes: Vec::new(),
        }
    }

    pub fn update(&mut self, joy: &Joy) {
        self.update_axes(joy.axes.as_slice());
        self.update_buttons(joy.buttons.as_slice());
    }

    pub fn update_buttons(&mut self, buttons: &[i32]) {
        self.previous_button_states = self.current_button_states.clone();
        self.current_button_states.clear();
        
        for (index, &state) in buttons.iter().enumerate() {
            self.current_button_states.insert(index, state != 0);
        }
    }

    pub fn update_axes(&mut self, axes: &[f32]) {
        self.previous_axes = self.current_axes.clone();
        self.current_axes = axes.to_vec();
    }

    pub fn get_axes(&self) -> &[f32] {
        &self.current_axes
    }

    pub fn get_axis(&self, axis_index: usize) -> Option<f32> {
        if self.current_axes.len() <= axis_index {
            return None; // Out of bounds
        }

        self.current_axes.get(axis_index).copied()
    }

    pub fn is_pressed(&self, button_index: usize) -> bool {
        if self.current_button_states.len() <= button_index {
            return false; // Out of bounds
        }

        *self.current_button_states.get(&button_index).unwrap_or(&false)
    }

    /// Check if a button was just pressed (rising edge)
    pub fn just_pressed(&self, button_index: usize) -> bool {
        let was_pressed = self
            .previous_button_states
            .get(&button_index)
            .unwrap_or(&false);
        let is_pressed = self
            .current_button_states
            .get(&button_index)
            .unwrap_or(&false);

        !was_pressed && *is_pressed
    }

    /// Check if a button was just released (falling edge)
    pub fn just_released(&self, button_index: usize) -> bool {
        let was_pressed = self
            .previous_button_states
            .get(&button_index)
            .unwrap_or(&false);
        let is_pressed = self
            .current_button_states
            .get(&button_index)
            .unwrap_or(&false);

        *was_pressed && !is_pressed
    }

    /// Check if a button is being held (was pressed and still is)
    pub fn is_held(&self, button_index: usize) -> bool {
        let was_pressed = self
            .previous_button_states
            .get(&button_index)
            .unwrap_or(&false);
        let is_pressed = self
            .current_button_states
            .get(&button_index)
            .unwrap_or(&false);
        *was_pressed && *is_pressed
    }

    pub fn get_pressed_buttons(&self) -> Vec<usize> {
        self.current_button_states
            .iter()
            .filter_map(|(&index, &pressed)| if pressed { Some(index) } else { None })
            .collect()
    }

    pub fn get_just_pressed_buttons(&self) -> Vec<usize> {
        self.current_button_states
            .iter()
            .filter_map(|(&index, &pressed)| {
                if pressed
                    && !self
                        .previous_button_states
                        .get(&index)
                        .unwrap_or(&false)
                {
                    Some(index)
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn are_all_pressed(&self, button_indices: &[usize]) -> bool {
        button_indices.iter().all(|&index| self.is_pressed(index))
    }

    pub fn is_any_pressed(&self, button_indices: &[usize]) -> bool {
        button_indices.iter().any(|&index| self.is_pressed(index))
    }

    pub fn button_count(&self) -> usize {
        self.current_button_states
            .len()
            .max(self.previous_button_states.len())
    }

    pub fn axes_count(&self) -> usize {
        self.current_axes.len()
    }
}

impl Default for JoyMsgTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_button_state_transitions() {
        let mut tracker = JoyMsgTracker::new();
        
        // Initial state - no buttons pressed
        assert!(!tracker.is_pressed(0));
        assert!(!tracker.just_pressed(0));
        assert!(!tracker.just_released(0));
        assert!(!tracker.is_held(0));
        
        // Press button 0
        tracker.update_buttons(&[1]);
        assert!(tracker.is_pressed(0));
        assert!(tracker.just_pressed(0));
        assert!(!tracker.just_released(0));
        assert!(!tracker.is_held(0));
        
        // Hold button 0
        tracker.update_buttons(&[1]);
        assert!(tracker.is_pressed(0));
        assert!(!tracker.just_pressed(0));
        assert!(!tracker.just_released(0));
        assert!(tracker.is_held(0));
        
        // Release button 0
        tracker.update_buttons(&[0]);
        assert!(!tracker.is_pressed(0));
        assert!(!tracker.just_pressed(0));
        assert!(tracker.just_released(0));
        assert!(!tracker.is_held(0));
    }

    #[test]
    fn test_multiple_button_tracking() {
        let mut tracker = JoyMsgTracker::new();
        
        // Press multiple buttons
        tracker.update_buttons(&[1, 0, 1, 1, 0]);
        
        assert!(tracker.is_pressed(0));
        assert!(!tracker.is_pressed(1));
        assert!(tracker.is_pressed(2));
        assert!(tracker.is_pressed(3));
        assert!(!tracker.is_pressed(4));
        
        let pressed = tracker.get_pressed_buttons();
        assert_eq!(pressed.len(), 3);
        assert!(pressed.contains(&0));
        assert!(pressed.contains(&2));
        assert!(pressed.contains(&3));
    }

    #[test]
    fn test_button_combinations() {
        let mut tracker = JoyMsgTracker::new();
        
        tracker.update_buttons(&[1, 0, 1, 0]);
        
        // Test are_all_pressed
        assert!(tracker.are_all_pressed(&[0, 2]));
        assert!(!tracker.are_all_pressed(&[0, 1, 2]));
        assert!(!tracker.are_all_pressed(&[1, 3]));
        
        // Test is_any_pressed
        assert!(tracker.is_any_pressed(&[0, 1]));
        assert!(tracker.is_any_pressed(&[1, 2, 3]));
        assert!(!tracker.is_any_pressed(&[1, 3]));
    }

    #[test]
    fn test_axes_tracking() {
        let mut tracker = JoyMsgTracker::new();
        
        // Initial state
        assert_eq!(tracker.get_axes(), &[] as &[f32]);
        assert_eq!(tracker.get_axis(0), None);
        
        // Update axes
        tracker.update_axes(&[0.5, -0.3, 0.0, 0.9]);
        
        assert_eq!(tracker.get_axes(), &[0.5, -0.3, 0.0, 0.9]);
        assert_eq!(tracker.get_axis(0), Some(0.5));
        assert_eq!(tracker.get_axis(1), Some(-0.3));
        assert_eq!(tracker.get_axis(2), Some(0.0));
        assert_eq!(tracker.get_axis(3), Some(0.9));
        assert_eq!(tracker.get_axis(4), None); // Out of bounds
        
        assert_eq!(tracker.axes_count(), 4);
    }

    #[test]
    fn test_update_from_joy_msg() {
        let mut tracker = JoyMsgTracker::new();
        let mut joy = Joy::new().unwrap();
        
        // Set up joy message using iter_mut
        joy.axes = safe_drive::msg::F32Seq::new(3).unwrap();
        joy.axes.iter_mut().enumerate().for_each(|(i, v)| {
            *v = match i {
                0 => 0.7,
                1 => -0.2,
                2 => 0.0,
                _ => 0.0,
            };
        });
        
        joy.buttons = safe_drive::msg::I32Seq::new(4).unwrap();
        joy.buttons.iter_mut().enumerate().for_each(|(i, v)| {
            *v = match i {
                0 => 1,
                2 => 1,
                _ => 0,
            };
        });
        
        // Update tracker
        tracker.update(&joy);
        
        // Verify axes
        assert_eq!(tracker.get_axis(0), Some(0.7));
        assert_eq!(tracker.get_axis(1), Some(-0.2));
        assert_eq!(tracker.get_axis(2), Some(0.0));
        
        // Verify buttons
        assert!(tracker.is_pressed(0));
        assert!(!tracker.is_pressed(1));
        assert!(tracker.is_pressed(2));
        assert!(!tracker.is_pressed(3));
    }

    #[test]
    fn test_just_pressed_sequence() {
        let mut tracker = JoyMsgTracker::new();
        
        // Frame 1: Press button 0 and 1
        tracker.update_buttons(&[1, 1, 0]);
        let just_pressed = tracker.get_just_pressed_buttons();
        assert_eq!(just_pressed.len(), 2);
        assert!(just_pressed.contains(&0));
        assert!(just_pressed.contains(&1));
        
        // Frame 2: Hold button 0, release 1, press 2
        tracker.update_buttons(&[1, 0, 1]);
        let just_pressed = tracker.get_just_pressed_buttons();
        assert_eq!(just_pressed.len(), 1);
        assert!(just_pressed.contains(&2));
        assert!(tracker.just_released(1));
        
        // Frame 3: Release all
        tracker.update_buttons(&[0, 0, 0]);
        let just_pressed = tracker.get_just_pressed_buttons();
        assert_eq!(just_pressed.len(), 0);
        assert!(tracker.just_released(0));
        assert!(tracker.just_released(2));
    }
}