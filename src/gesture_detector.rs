use std::time::{Duration, Instant};
use std::collections::{HashMap, VecDeque};
use serde::{Deserialize, Serialize};
use crate::button_tracker::ButtonTracker;
use crate::error::JoyRouterResult;
use crate::config::ActionType;
use safe_drive::logger::Logger;
use safe_drive::{pr_debug, pr_info};

/// Defines a gesture pattern
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GestureDefinition {
    /// Name of the gesture
    pub name: String,
    /// Description of the gesture
    #[serde(default)]
    pub description: String,
    /// Pattern to match
    pub pattern: GesturePattern,
    /// Action to execute when gesture is detected
    pub action: ActionType,
    /// Minimum duration the gesture must be held to trigger
    #[serde(default)]
    pub min_duration_ms: u64,
    /// Maximum time between gesture steps
    #[serde(default = "default_max_step_time")]
    pub max_step_time_ms: u64,
    /// Whether the gesture can repeat while held
    #[serde(default)]
    pub repeatable: bool,
    /// Repeat interval if repeatable
    #[serde(default = "default_repeat_interval")]
    pub repeat_interval_ms: u64,
}

/// Types of gesture patterns
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum GesturePattern {
    /// Long press gesture
    LongPress {
        button: usize,
        duration_ms: u64,
    },
    /// Multi-tap gesture (double-tap, triple-tap, etc.)
    MultiTap {
        button: usize,
        tap_count: usize,
        max_tap_interval_ms: u64,
    },
    /// Button sequence gesture
    Sequence {
        buttons: Vec<usize>,
        max_interval_ms: u64,
    },
    /// Chord gesture (multiple buttons pressed simultaneously)
    Chord {
        buttons: Vec<usize>,
        require_all: bool,
        min_duration_ms: u64,
    },
    /// Rhythm gesture (specific timing pattern)
    Rhythm {
        button: usize,
        pattern: Vec<u64>, // Intervals in milliseconds
        tolerance_ms: u64,
    },
    /// Directional gesture using D-pad or multiple buttons
    Directional {
        direction: Direction,
        buttons: DirectionButtons,
    },
}

/// Direction for directional gestures
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Direction {
    Up,
    Down,
    Left,
    Right,
    UpLeft,
    UpRight,
    DownLeft,
    DownRight,
    Circle,      // Circular motion
    Figure8,     // Figure-8 pattern
}

/// Button mappings for directional gestures
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DirectionButtons {
    pub up: usize,
    pub down: usize,
    pub left: usize,
    pub right: usize,
}

/// State of gesture detection
#[derive(Debug)]
pub struct GestureState {
    /// When the gesture started
    pub start_time: Instant,
    /// Current step in the gesture
    pub current_step: usize,
    /// Button press history for pattern matching
    pub press_history: VecDeque<ButtonPressEvent>,
    /// Last time the gesture was triggered (for repeatables)
    pub last_trigger_time: Option<Instant>,
    /// Whether the gesture is currently active
    pub active: bool,
}

/// A button press event with timing
#[derive(Debug, Clone)]
pub struct ButtonPressEvent {
    pub button: usize,
    pub timestamp: Instant,
    pub event_type: ButtonEventType,
}

/// Type of button event
#[derive(Debug, Clone, PartialEq)]
pub enum ButtonEventType {
    Press,
    Release,
}

/// Manages gesture detection
pub struct GestureDetector {
    /// Gesture definitions
    gestures: HashMap<String, GestureDefinition>,
    /// Current gesture states
    gesture_states: HashMap<String, GestureState>,
    /// Long press tracking
    long_press_tracker: HashMap<usize, Instant>,
    /// Logger
    logger: Logger,
}

impl GestureDetector {
    /// Create a new gesture detector
    pub fn new(gestures: HashMap<String, GestureDefinition>) -> Self {
        let logger = Logger::new("gesture_detector");
        pr_info!(logger, "Created gesture detector with {} gestures", gestures.len());
        
        Self {
            gestures,
            gesture_states: HashMap::new(),
            long_press_tracker: HashMap::new(),
            logger,
        }
    }
    
    /// Update gesture detection with current button state
    pub fn update(&mut self, button_tracker: &ButtonTracker) -> JoyRouterResult<Vec<(String, ActionType)>> {
        let now = Instant::now();
        let mut triggered_gestures = Vec::new();
        
        // Update long press tracking
        self.update_long_press_tracking(button_tracker, now);
        
        // Update button press history
        self.update_press_history(button_tracker, now);
        
        // Check each gesture for matches
        let gesture_names: Vec<_> = self.gestures.keys().cloned().collect();
        for gesture_name in gesture_names {
            if let Some(gesture_def) = self.gestures.get(&gesture_name).cloned() {
                if let Some(action) = self.check_gesture(&gesture_name, &gesture_def, button_tracker, now)? {
                    triggered_gestures.push((gesture_name, action));
                }
            }
        }
        
        // Clean up old gesture states
        self.cleanup_old_states(now);
        
        Ok(triggered_gestures)
    }
    
    /// Update long press tracking
    fn update_long_press_tracking(&mut self, button_tracker: &ButtonTracker, now: Instant) {
        // Track new button presses
        for button_idx in 0..16 { // Assume max 16 buttons
            if button_tracker.just_pressed(button_idx) {
                self.long_press_tracker.insert(button_idx, now);
            } else if button_tracker.just_released(button_idx) {
                self.long_press_tracker.remove(&button_idx);
            }
        }
    }
    
    /// Update button press history
    fn update_press_history(&mut self, button_tracker: &ButtonTracker, now: Instant) {
        // Record new button events
        for button_idx in 0..16 { // Assume max 16 buttons
            if button_tracker.just_pressed(button_idx) {
                // Add press event to all active gesture states
                for state in self.gesture_states.values_mut() {
                    state.press_history.push_back(ButtonPressEvent {
                        button: button_idx,
                        timestamp: now,
                        event_type: ButtonEventType::Press,
                    });
                    
                    // Limit history size
                    if state.press_history.len() > 20 {
                        state.press_history.pop_front();
                    }
                }
            }
            
            if button_tracker.just_released(button_idx) {
                // Add release event to all active gesture states
                for state in self.gesture_states.values_mut() {
                    state.press_history.push_back(ButtonPressEvent {
                        button: button_idx,
                        timestamp: now,
                        event_type: ButtonEventType::Release,
                    });
                    
                    // Limit history size
                    if state.press_history.len() > 20 {
                        state.press_history.pop_front();
                    }
                }
            }
        }
    }
    
    /// Check if a specific gesture matches current state
    fn check_gesture(
        &mut self, 
        gesture_name: &str, 
        gesture_def: &GestureDefinition,
        button_tracker: &ButtonTracker,
        now: Instant
    ) -> JoyRouterResult<Option<ActionType>> {
        // Get or create gesture state
        let state = self.gesture_states.entry(gesture_name.to_string())
            .or_insert_with(|| GestureState {
                start_time: now,
                current_step: 0,
                press_history: VecDeque::new(),
                last_trigger_time: None,
                active: false,
            });
        
        match &gesture_def.pattern {
            GesturePattern::LongPress { button, duration_ms } => {
                Self::check_long_press_gesture_static(&self.long_press_tracker, *button, *duration_ms, button_tracker, now, state, gesture_def)
            }
            GesturePattern::MultiTap { button, tap_count, max_tap_interval_ms } => {
                let logger = Logger::new("gesture_detector");
                Self::check_multi_tap_gesture_static(*button, *tap_count, *max_tap_interval_ms, now, state, gesture_def, &logger)
            }
            GesturePattern::Sequence { buttons, max_interval_ms } => {
                let logger = Logger::new("gesture_detector");
                Self::check_sequence_gesture_static(buttons, *max_interval_ms, now, state, gesture_def, &logger)
            }
            GesturePattern::Chord { buttons, require_all, min_duration_ms } => {
                let logger = Logger::new("gesture_detector");
                Self::check_chord_gesture_static(buttons, *require_all, *min_duration_ms, button_tracker, now, state, gesture_def, &logger)
            }
            GesturePattern::Rhythm { button, pattern, tolerance_ms } => {
                let logger = Logger::new("gesture_detector");
                Self::check_rhythm_gesture_static(*button, pattern, *tolerance_ms, now, state, gesture_def, &logger)
            }
            GesturePattern::Directional { direction, buttons } => {
                let logger = Logger::new("gesture_detector");
                Self::check_directional_gesture_static(direction, buttons, button_tracker, now, state, gesture_def, &logger)
            }
        }
    }
    
    /// Check long press gesture
    fn check_long_press_gesture_static(
        long_press_tracker: &HashMap<usize, Instant>,
        button: usize,
        duration_ms: u64,
        button_tracker: &ButtonTracker,
        now: Instant,
        state: &mut GestureState,
        gesture_def: &GestureDefinition
    ) -> JoyRouterResult<Option<ActionType>> {
        if let Some(press_start) = long_press_tracker.get(&button) {
            let hold_duration = now.duration_since(*press_start);
            let required_duration = Duration::from_millis(duration_ms);
            
            if hold_duration >= required_duration && button_tracker.is_pressed(button) {
                // Check if this is a repeatable gesture
                if gesture_def.repeatable {
                    if let Some(last_trigger) = state.last_trigger_time {
                        let repeat_interval = Duration::from_millis(gesture_def.repeat_interval_ms);
                        if now.duration_since(last_trigger) >= repeat_interval {
                            state.last_trigger_time = Some(now);
                            let logger = Logger::new("gesture_detector");
                            pr_debug!(logger, "Long press gesture '{}' repeated", gesture_def.name);
                            return Ok(Some(gesture_def.action.clone()));
                        }
                    } else {
                        state.last_trigger_time = Some(now);
                        let logger = Logger::new("gesture_detector");
                        pr_info!(logger, "Long press gesture '{}' triggered", gesture_def.name);
                        return Ok(Some(gesture_def.action.clone()));
                    }
                } else if state.last_trigger_time.is_none() {
                    state.last_trigger_time = Some(now);
                    let logger = Logger::new("gesture_detector");
                    pr_info!(logger, "Long press gesture '{}' triggered", gesture_def.name);
                    return Ok(Some(gesture_def.action.clone()));
                }
            }
        } else if state.last_trigger_time.is_some() {
            // Button was released, reset trigger state
            state.last_trigger_time = None;
        }
        
        Ok(None)
    }
    
    /// Check multi-tap gesture
    fn check_multi_tap_gesture_static(
        button: usize,
        tap_count: usize,
        max_tap_interval_ms: u64,
        now: Instant,
        state: &mut GestureState,
        gesture_def: &GestureDefinition,
        logger: &Logger
    ) -> JoyRouterResult<Option<ActionType>> {
        // Count recent button presses for the specified button
        let max_interval = Duration::from_millis(max_tap_interval_ms);
        let mut recent_presses = 0;
        let mut last_press_time = None;
        
        for event in state.press_history.iter().rev() {
            if event.button == button && event.event_type == ButtonEventType::Press {
                if now.duration_since(event.timestamp) <= max_interval {
                    recent_presses += 1;
                    if last_press_time.is_none() {
                        last_press_time = Some(event.timestamp);
                    }
                } else {
                    break;
                }
            }
        }
        
        if recent_presses >= tap_count {
            // Check if we haven't already triggered for this sequence
            if let Some(last_press) = last_press_time {
                if state.last_trigger_time.map_or(true, |t| last_press > t) {
                    state.last_trigger_time = Some(now);
                    pr_info!(logger, "Multi-tap gesture '{}' triggered ({} taps)", gesture_def.name, tap_count);
                    return Ok(Some(gesture_def.action.clone()));
                }
            }
        }
        
        Ok(None)
    }
    
    /// Check sequence gesture
    fn check_sequence_gesture_static(
        buttons: &[usize],
        max_interval_ms: u64,
        now: Instant,
        state: &mut GestureState,
        gesture_def: &GestureDefinition,
        logger: &Logger
    ) -> JoyRouterResult<Option<ActionType>> {
        let max_interval = Duration::from_millis(max_interval_ms);
        
        // Check if the recent button presses match the sequence
        if state.press_history.len() >= buttons.len() {
            let recent_events: Vec<_> = state.press_history.iter()
                .rev()
                .take(buttons.len())
                .filter(|e| e.event_type == ButtonEventType::Press)
                .collect();
            
            if recent_events.len() >= buttons.len() {
                let mut sequence_matches = true;
                let mut sequence_start_time = None;
                
                for (i, &expected_button) in buttons.iter().enumerate() {
                    if let Some(event) = recent_events.get(buttons.len() - 1 - i) {
                        if event.button != expected_button {
                            sequence_matches = false;
                            break;
                        }
                        if sequence_start_time.is_none() {
                            sequence_start_time = Some(event.timestamp);
                        }
                        // Check timing between steps
                        if i > 0 {
                            if let Some(prev_event) = recent_events.get(buttons.len() - i) {
                                if event.timestamp.duration_since(prev_event.timestamp) > max_interval {
                                    sequence_matches = false;
                                    break;
                                }
                            }
                        }
                    } else {
                        sequence_matches = false;
                        break;
                    }
                }
                
                if sequence_matches {
                    if let Some(start_time) = sequence_start_time {
                        if state.last_trigger_time.map_or(true, |t| start_time > t) {
                            state.last_trigger_time = Some(now);
                            pr_info!(logger, "Sequence gesture '{}' triggered", gesture_def.name);
                            return Ok(Some(gesture_def.action.clone()));
                        }
                    }
                }
            }
        }
        
        Ok(None)
    }
    
    /// Check chord gesture
    fn check_chord_gesture_static(
        buttons: &[usize],
        require_all: bool,
        min_duration_ms: u64,
        button_tracker: &ButtonTracker,
        now: Instant,
        state: &mut GestureState,
        gesture_def: &GestureDefinition,
        logger: &Logger
    ) -> JoyRouterResult<Option<ActionType>> {
        let buttons_match = if require_all {
            buttons.iter().all(|&btn| button_tracker.is_pressed(btn))
        } else {
            buttons.iter().any(|&btn| button_tracker.is_pressed(btn))
        };
        
        if buttons_match {
            if !state.active {
                state.active = true;
                state.start_time = now;
            }
            
            let hold_duration = now.duration_since(state.start_time);
            let required_duration = Duration::from_millis(min_duration_ms);
            
            if hold_duration >= required_duration {
                if state.last_trigger_time.is_none() {
                    state.last_trigger_time = Some(now);
                    pr_info!(logger, "Chord gesture '{}' triggered", gesture_def.name);
                    return Ok(Some(gesture_def.action.clone()));
                }
            }
        } else {
            state.active = false;
            state.last_trigger_time = None;
        }
        
        Ok(None)
    }
    
    /// Check rhythm gesture
    fn check_rhythm_gesture_static(
        button: usize,
        pattern: &[u64],
        tolerance_ms: u64,
        now: Instant,
        state: &mut GestureState,
        gesture_def: &GestureDefinition,
        logger: &Logger
    ) -> JoyRouterResult<Option<ActionType>> {
        // Find recent button presses for this button
        let button_presses: Vec<_> = state.press_history.iter()
            .filter(|e| e.button == button && e.event_type == ButtonEventType::Press)
            .collect();
        
        if button_presses.len() >= pattern.len() + 1 {
            let recent_presses: Vec<_> = button_presses.iter()
                .rev()
                .take(pattern.len() + 1)
                .cloned()
                .collect();
            
            let mut rhythm_matches = true;
            for (i, &expected_interval) in pattern.iter().enumerate() {
                if let (Some(curr), Some(prev)) = (recent_presses.get(i), recent_presses.get(i + 1)) {
                    let actual_interval = curr.timestamp.duration_since(prev.timestamp).as_millis() as u64;
                    let tolerance = tolerance_ms;
                    
                    if actual_interval < expected_interval.saturating_sub(tolerance) ||
                       actual_interval > expected_interval + tolerance {
                        rhythm_matches = false;
                        break;
                    }
                } else {
                    rhythm_matches = false;
                    break;
                }
            }
            
            if rhythm_matches {
                if let Some(latest_press) = recent_presses.first() {
                    if state.last_trigger_time.map_or(true, |t| latest_press.timestamp > t) {
                        state.last_trigger_time = Some(now);
                        pr_info!(logger, "Rhythm gesture '{}' triggered", gesture_def.name);
                        return Ok(Some(gesture_def.action.clone()));
                    }
                }
            }
        }
        
        Ok(None)
    }
    
    /// Check directional gesture
    fn check_directional_gesture_static(
        direction: &Direction,
        buttons: &DirectionButtons,
        button_tracker: &ButtonTracker,
        now: Instant,
        state: &mut GestureState,
        gesture_def: &GestureDefinition,
        logger: &Logger
    ) -> JoyRouterResult<Option<ActionType>> {
        // Simple directional detection - can be extended for complex patterns
        let direction_pressed = match direction {
            Direction::Up => button_tracker.is_pressed(buttons.up),
            Direction::Down => button_tracker.is_pressed(buttons.down),
            Direction::Left => button_tracker.is_pressed(buttons.left),
            Direction::Right => button_tracker.is_pressed(buttons.right),
            Direction::UpLeft => button_tracker.is_pressed(buttons.up) && button_tracker.is_pressed(buttons.left),
            Direction::UpRight => button_tracker.is_pressed(buttons.up) && button_tracker.is_pressed(buttons.right),
            Direction::DownLeft => button_tracker.is_pressed(buttons.down) && button_tracker.is_pressed(buttons.left),
            Direction::DownRight => button_tracker.is_pressed(buttons.down) && button_tracker.is_pressed(buttons.right),
            Direction::Circle | Direction::Figure8 => {
                // Complex patterns would require sequence analysis
                false
            }
        };
        
        if direction_pressed && state.last_trigger_time.is_none() {
            state.last_trigger_time = Some(now);
            pr_info!(logger, "Directional gesture '{}' triggered", gesture_def.name);
            return Ok(Some(gesture_def.action.clone()));
        } else if !direction_pressed {
            state.last_trigger_time = None;
        }
        
        Ok(None)
    }
    
    /// Clean up old gesture states
    fn cleanup_old_states(&mut self, now: Instant) {
        let cleanup_threshold = Duration::from_secs(30);
        
        self.gesture_states.retain(|_, state| {
            now.duration_since(state.start_time) < cleanup_threshold
        });
        
        // Clean up old events from press history
        for state in self.gesture_states.values_mut() {
            while let Some(front) = state.press_history.front() {
                if now.duration_since(front.timestamp) > cleanup_threshold {
                    state.press_history.pop_front();
                } else {
                    break;
                }
            }
        }
    }
    
    /// Get status of all gesture detectors
    pub fn get_status(&self) -> Vec<(String, bool)> {
        self.gestures.iter()
            .map(|(name, _)| {
                let active = self.gesture_states.get(name)
                    .map_or(false, |state| state.active);
                (name.clone(), active)
            })
            .collect()
    }
}

fn default_max_step_time() -> u64 {
    1000 // 1 second
}

fn default_repeat_interval() -> u64 {
    500 // 500ms
}

#[cfg(test)]
mod tests {
    use super::*;
    
    fn create_test_gestures() -> HashMap<String, GestureDefinition> {
        let mut gestures = HashMap::new();
        
        gestures.insert("long_press_test".to_string(), GestureDefinition {
            name: "long_press_test".to_string(),
            description: "Test long press".to_string(),
            pattern: GesturePattern::LongPress {
                button: 0,
                duration_ms: 1000,
            },
            action: ActionType::NoAction,
            min_duration_ms: 0,
            max_step_time_ms: 1000,
            repeatable: false,
            repeat_interval_ms: 500,
        });
        
        gestures.insert("double_tap".to_string(), GestureDefinition {
            name: "double_tap".to_string(),
            description: "Double tap gesture".to_string(),
            pattern: GesturePattern::MultiTap {
                button: 1,
                tap_count: 2,
                max_tap_interval_ms: 500,
            },
            action: ActionType::NoAction,
            min_duration_ms: 0,
            max_step_time_ms: 1000,
            repeatable: false,
            repeat_interval_ms: 500,
        });
        
        gestures
    }
    
    #[test]
    fn test_gesture_detector_creation() {
        let gestures = create_test_gestures();
        let detector = GestureDetector::new(gestures);
        
        let status = detector.get_status();
        assert_eq!(status.len(), 2);
    }
    
    #[test]
    fn test_gesture_patterns() {
        let long_press = GesturePattern::LongPress {
            button: 0,
            duration_ms: 1000,
        };
        
        let multi_tap = GesturePattern::MultiTap {
            button: 1,
            tap_count: 3,
            max_tap_interval_ms: 300,
        };
        
        let sequence = GesturePattern::Sequence {
            buttons: vec![0, 1, 2],
            max_interval_ms: 500,
        };
        
        match long_press {
            GesturePattern::LongPress { button, duration_ms } => {
                assert_eq!(button, 0);
                assert_eq!(duration_ms, 1000);
            }
            _ => panic!("Expected LongPress"),
        }
        
        match multi_tap {
            GesturePattern::MultiTap { button, tap_count, max_tap_interval_ms } => {
                assert_eq!(button, 1);
                assert_eq!(tap_count, 3);
                assert_eq!(max_tap_interval_ms, 300);
            }
            _ => panic!("Expected MultiTap"),
        }
        
        match sequence {
            GesturePattern::Sequence { buttons, max_interval_ms } => {
                assert_eq!(buttons, vec![0, 1, 2]);
                assert_eq!(max_interval_ms, 500);
            }
            _ => panic!("Expected Sequence"),
        }
    }
    
    #[test]
    fn test_button_press_event() {
        let now = Instant::now();
        let event = ButtonPressEvent {
            button: 5,
            timestamp: now,
            event_type: ButtonEventType::Press,
        };
        
        assert_eq!(event.button, 5);
        assert_eq!(event.event_type, ButtonEventType::Press);
    }
    
    #[test]
    fn test_direction_buttons() {
        let direction_buttons = DirectionButtons {
            up: 12,
            down: 13,
            left: 14,
            right: 15,
        };
        
        assert_eq!(direction_buttons.up, 12);
        assert_eq!(direction_buttons.down, 13);
        assert_eq!(direction_buttons.left, 14);
        assert_eq!(direction_buttons.right, 15);
    }
}