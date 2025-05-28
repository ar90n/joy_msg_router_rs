use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std::collections::HashMap;

/// Timer callback that can be registered and called periodically
pub trait TimerCallback: Send + Sync {
    fn on_timer(&mut self);
}

/// Timer event with ID and callback
pub struct TimerEvent {
    pub id: String,
    pub interval: Duration,
    pub callback: Box<dyn TimerCallback>,
    last_triggered: Instant,
}

impl TimerEvent {
    pub fn new(id: String, interval: Duration, callback: Box<dyn TimerCallback>) -> Self {
        Self {
            id,
            interval,
            callback,
            last_triggered: Instant::now(),
        }
    }
    
    /// Check if timer should trigger and execute callback if needed
    pub fn check_and_trigger(&mut self) -> bool {
        let now = Instant::now();
        if now.duration_since(self.last_triggered) >= self.interval {
            self.callback.on_timer();
            self.last_triggered = now;
            true
        } else {
            false
        }
    }
}

/// Timer manager that handles multiple periodic timers
pub struct TimerManager {
    timers: Arc<Mutex<HashMap<String, TimerEvent>>>,
}

impl TimerManager {
    pub fn new() -> Self {
        Self {
            timers: Arc::new(Mutex::new(HashMap::new())),
        }
    }
    
    /// Register a new timer with given ID and interval
    pub fn register_timer(&self, id: String, interval: Duration, callback: Box<dyn TimerCallback>) {
        let mut timers = self.timers.lock().unwrap();
        timers.insert(id.clone(), TimerEvent::new(id, interval, callback));
    }
    
    /// Unregister a timer by ID
    pub fn unregister_timer(&self, id: &str) -> bool {
        let mut timers = self.timers.lock().unwrap();
        timers.remove(id).is_some()
    }
    
    /// Process all timers and trigger callbacks as needed
    pub fn process_timers(&self) {
        let mut timers = self.timers.lock().unwrap();
        for (_, timer) in timers.iter_mut() {
            timer.check_and_trigger();
        }
    }
    
    /// Get the minimum interval among all timers for efficient sleeping
    pub fn get_min_interval(&self) -> Option<Duration> {
        let timers = self.timers.lock().unwrap();
        timers.values()
            .map(|t| t.interval)
            .min()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    
    struct TestCallback {
        counter: Arc<AtomicUsize>,
    }
    
    impl TimerCallback for TestCallback {
        fn on_timer(&mut self) {
            self.counter.fetch_add(1, Ordering::SeqCst);
        }
    }
    
    #[test]
    fn test_timer_event_trigger() {
        let counter = Arc::new(AtomicUsize::new(0));
        let callback = Box::new(TestCallback { counter: counter.clone() });
        let mut timer = TimerEvent::new("test".to_string(), Duration::from_millis(100), callback);
        
        // Should not trigger immediately
        assert!(!timer.check_and_trigger());
        assert_eq!(counter.load(Ordering::SeqCst), 0);
        
        // Wait and trigger
        std::thread::sleep(Duration::from_millis(150));
        assert!(timer.check_and_trigger());
        assert_eq!(counter.load(Ordering::SeqCst), 1);
    }
    
    #[test]
    fn test_timer_manager() {
        let manager = TimerManager::new();
        let counter1 = Arc::new(AtomicUsize::new(0));
        let counter2 = Arc::new(AtomicUsize::new(0));
        
        // Register two timers
        manager.register_timer(
            "timer1".to_string(),
            Duration::from_millis(50),
            Box::new(TestCallback { counter: counter1.clone() })
        );
        
        manager.register_timer(
            "timer2".to_string(),
            Duration::from_millis(100),
            Box::new(TestCallback { counter: counter2.clone() })
        );
        
        // Check minimum interval
        assert_eq!(manager.get_min_interval(), Some(Duration::from_millis(50)));
        
        // Process timers multiple times
        std::thread::sleep(Duration::from_millis(60));
        manager.process_timers();
        assert_eq!(counter1.load(Ordering::SeqCst), 1);
        assert_eq!(counter2.load(Ordering::SeqCst), 0);
        
        std::thread::sleep(Duration::from_millis(60));
        manager.process_timers();
        assert_eq!(counter1.load(Ordering::SeqCst), 2);
        assert_eq!(counter2.load(Ordering::SeqCst), 1);
        
        // Unregister timer1
        assert!(manager.unregister_timer("timer1"));
        assert!(!manager.unregister_timer("timer1")); // Already removed
        
        // Check minimum interval after removal
        assert_eq!(manager.get_min_interval(), Some(Duration::from_millis(100)));
    }
}