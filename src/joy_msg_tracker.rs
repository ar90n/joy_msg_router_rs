use sensor_msgs::msg::Joy;
use std::collections::HashMap;

/// Tracks joy message states (axes and buttons) and detects press/release events
#[derive(Debug, Clone)]
pub struct JoyMsgTracker {
    /// Previous frame button states
    previous_button_states: HashMap<usize, bool>,
    /// Current frame button states
    current_button_states: HashMap<usize, bool>,
    /// Current axes values
    current_axes: Vec<f32>,
    /// Previous axes values
    previous_axes: Vec<f32>,
}

impl JoyMsgTracker {
    /// Create a new joy message tracker
    pub fn new() -> Self {
        Self {
            previous_button_states: HashMap::new(),
            current_button_states: HashMap::new(),
            current_axes: Vec::new(),
            previous_axes: Vec::new(),
        }
    }

    /// Update states from a Joy message
    pub fn update(&mut self, joy_msg: &Joy) {
        // Update button states
        self.update_buttons(joy_msg.buttons.as_slice());

        // Update axes states
        self.update_axes(joy_msg.axes.as_slice());
    }

    /// Update only button states from button array
    pub fn update_buttons(&mut self, buttons: &[i32]) {
        // Move current to previous
        self.previous_button_states = self.current_button_states.clone();

        // Clear current states
        self.current_button_states.clear();

        // Update current states from button array
        for (index, &value) in buttons.iter().enumerate() {
            self.current_button_states.insert(index, value != 0);
        }
    }

    /// Update only axes states from axes array
    pub fn update_axes(&mut self, axes: &[f32]) {
        // Move current to previous
        self.previous_axes = self.current_axes.clone();

        // Update current axes
        self.current_axes = axes.to_vec();
    }

    /// Get current axes values
    pub fn get_axes(&self) -> &[f32] {
        &self.current_axes
    }

    /// Get a specific axis value
    pub fn get_axis(&self, axis_index: usize) -> Option<f32> {
        self.current_axes.get(axis_index).copied()
    }

    /// Check if a button is currently pressed
    pub fn is_pressed(&self, button_index: usize) -> bool {
        self.current_button_states
            .get(&button_index)
            .copied()
            .unwrap_or(false)
    }

    /// Check if a button was just pressed (rising edge)
    pub fn just_pressed(&self, button_index: usize) -> bool {
        let was_pressed = self
            .previous_button_states
            .get(&button_index)
            .copied()
            .unwrap_or(false);
        let is_pressed = self
            .current_button_states
            .get(&button_index)
            .copied()
            .unwrap_or(false);
        !was_pressed && is_pressed
    }

    /// Check if a button was just released (falling edge)
    pub fn just_released(&self, button_index: usize) -> bool {
        let was_pressed = self
            .previous_button_states
            .get(&button_index)
            .copied()
            .unwrap_or(false);
        let is_pressed = self
            .current_button_states
            .get(&button_index)
            .copied()
            .unwrap_or(false);
        was_pressed && !is_pressed
    }

    /// Check if a button is being held (was pressed and still is)
    pub fn is_held(&self, button_index: usize) -> bool {
        let was_pressed = self
            .previous_button_states
            .get(&button_index)
            .copied()
            .unwrap_or(false);
        let is_pressed = self
            .current_button_states
            .get(&button_index)
            .copied()
            .unwrap_or(false);
        was_pressed && is_pressed
    }

    /// Get all currently pressed button indices
    pub fn get_pressed_buttons(&self) -> Vec<usize> {
        self.current_button_states
            .iter()
            .filter_map(|(&index, &pressed)| if pressed { Some(index) } else { None })
            .collect()
    }

    /// Get all buttons that were just pressed
    pub fn get_just_pressed_buttons(&self) -> Vec<usize> {
        self.current_button_states
            .iter()
            .filter_map(|(&index, &pressed)| {
                if pressed
                    && !self
                        .previous_button_states
                        .get(&index)
                        .copied()
                        .unwrap_or(false)
                {
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

    /// Get the number of buttons being tracked
    pub fn button_count(&self) -> usize {
        self.current_button_states
            .len()
            .max(self.previous_button_states.len())
    }

    /// Get the number of axes being tracked
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

    fn create_test_joy(axes: Vec<f32>, buttons: Vec<i32>) -> Joy {
        let mut joy = Joy::new().unwrap();
        joy.axes = safe_drive::msg::F32Seq::new(axes.len()).unwrap();
        joy.axes
            .iter_mut()
            .enumerate()
            .for_each(|(i, v)| *v = axes[i]);
        joy.buttons = safe_drive::msg::I32Seq::new(buttons.len()).unwrap();
        joy.buttons
            .iter_mut()
            .enumerate()
            .for_each(|(i, v)| *v = buttons[i]);
        joy
    }

    #[test]
    fn test_button_press_detection() {
        let mut tracker = JoyMsgTracker::new();

        // Initial state - no buttons pressed
        tracker.update_buttons(&[0, 0, 0]);
        assert!(!tracker.is_pressed(0));
        assert!(!tracker.just_pressed(0));
        assert!(!tracker.just_released(0));

        // Press button 0
        tracker.update_buttons(&[1, 0, 0]);
        assert!(tracker.is_pressed(0));
        assert!(tracker.just_pressed(0));
        assert!(!tracker.just_released(0));
        assert!(!tracker.is_held(0));

        // Hold button 0
        tracker.update_buttons(&[1, 0, 0]);
        assert!(tracker.is_pressed(0));
        assert!(!tracker.just_pressed(0));
        assert!(!tracker.just_released(0));
        assert!(tracker.is_held(0));

        // Release button 0
        tracker.update_buttons(&[0, 0, 0]);
        assert!(!tracker.is_pressed(0));
        assert!(!tracker.just_pressed(0));
        assert!(tracker.just_released(0));
        assert!(!tracker.is_held(0));
    }

    #[test]
    fn test_multiple_buttons() {
        let mut tracker = JoyMsgTracker::new();

        // Press multiple buttons
        tracker.update_buttons(&[1, 0, 1, 0, 1]);
        let mut pressed = tracker.get_pressed_buttons();
        pressed.sort();
        assert_eq!(pressed, vec![0, 2, 4]);

        let mut just_pressed = tracker.get_just_pressed_buttons();
        just_pressed.sort();
        assert_eq!(just_pressed, vec![0, 2, 4]);

        // Hold some, release others, press new ones
        tracker.update_buttons(&[1, 1, 0, 0, 1]);
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
        let mut tracker = JoyMsgTracker::new();

        tracker.update_buttons(&[1, 1, 0, 1]);
        assert!(tracker.are_all_pressed(&[0, 1, 3]));
        assert!(!tracker.are_all_pressed(&[0, 1, 2, 3]));
        assert!(tracker.is_any_pressed(&[0, 2]));
        assert!(!tracker.is_any_pressed(&[2, 5]));
    }

    #[test]
    fn test_expanding_button_array() {
        let mut tracker = JoyMsgTracker::new();

        // Start with 3 buttons
        tracker.update_buttons(&[1, 0, 1]);
        assert!(tracker.is_pressed(0));
        assert!(tracker.is_pressed(2));

        // Expand to 5 buttons
        tracker.update_buttons(&[1, 0, 1, 0, 1]);
        assert!(tracker.is_pressed(0));
        assert!(tracker.is_pressed(2));
        assert!(tracker.just_pressed(4));

        // Shrink back to 2 buttons
        tracker.update_buttons(&[0, 1]);
        assert!(!tracker.is_pressed(0));
        assert!(tracker.is_pressed(1));
        assert!(!tracker.is_pressed(2)); // No longer exists
    }

    #[test]
    fn test_axes_tracking() {
        let mut tracker = JoyMsgTracker::new();

        // Initial axes
        tracker.update_axes(&[0.0, 0.5, -0.3]);
        assert_eq!(tracker.get_axes(), &[0.0, 0.5, -0.3]);
        assert_eq!(tracker.get_axis(1), Some(0.5));
        assert_eq!(tracker.get_axis(5), None);
        assert_eq!(tracker.axes_count(), 3);

        // Update axes
        tracker.update_axes(&[0.2, 0.7, -0.1, 0.9]);
        assert_eq!(tracker.get_axes(), &[0.2, 0.7, -0.1, 0.9]);
        assert_eq!(tracker.axes_count(), 4);
    }

    #[test]
    fn test_full_joy_update() {
        let mut tracker = JoyMsgTracker::new();

        // Create test joy message
        let joy1 = create_test_joy(vec![0.0, 0.5], vec![0, 1, 0]);
        tracker.update(&joy1);

        assert_eq!(tracker.get_axes(), &[0.0, 0.5]);
        assert!(tracker.is_pressed(1));
        assert!(!tracker.is_pressed(0));

        // Update with new joy message
        let joy2 = create_test_joy(vec![0.3, 0.7], vec![1, 1, 0]);
        tracker.update(&joy2);

        assert_eq!(tracker.get_axes(), &[0.3, 0.7]);
        assert!(tracker.is_pressed(0));
        assert!(tracker.is_pressed(1));
        assert!(tracker.just_pressed(0));
        assert!(!tracker.just_pressed(1)); // Was already pressed
    }
}
