# Development Roadmap for joy_msg_router_rs

## Overview
This document outlines the planned development path for new features in the joy_msg_router_rs project.

## Development Phases

### Phase 1: Core Architecture Improvements (High Priority)
These features establish the foundation for more advanced functionality.

#### Issue #18: Command Queue Architecture
- **Priority**: High
- **Description**: Implement command queue system for decoupled action processing
- **Dependencies**: None
- **Estimated Effort**: Medium

#### Issue #19: Timer Callback System  
- **Priority**: High
- **Description**: Add timer-based periodic command processing
- **Dependencies**: #18 (command queue)
- **Estimated Effort**: Medium

### Phase 2: Enhanced Functionality (Medium Priority)
Build on the core architecture to add powerful new features.

#### Issue #16: Multiple Output Message Types
- **Priority**: Medium
- **Description**: Support various ROS2 message types beyond Twist
- **Dependencies**: #18
- **Estimated Effort**: Medium

#### Issue #20: Action Sequences and Macros
- **Priority**: Medium  
- **Description**: Enable complex multi-step actions from single button press
- **Dependencies**: #18, #19
- **Estimated Effort**: High

#### Issue #22: Gesture Detection
- **Priority**: Medium
- **Description**: Detect long press, double-click, and button combinations
- **Dependencies**: #19 (for timing)
- **Estimated Effort**: Medium

### Phase 3: Advanced Features (Lower Priority)
Sophisticated features for complex use cases.

#### Issue #21: State Machine
- **Priority**: Low
- **Description**: Modal behavior and complex state management
- **Dependencies**: #18, #19, #20
- **Estimated Effort**: High

#### Issue #23: Dynamic Reconfiguration
- **Priority**: Low
- **Description**: Runtime parameter updates via ROS2 parameter server
- **Dependencies**: None (but benefits from #18)
- **Estimated Effort**: Medium

#### Issue #17: Enhanced Logging
- **Priority**: Low
- **Description**: Structured logging and diagnostics
- **Dependencies**: None
- **Estimated Effort**: Low

## Implementation Order

1. **Start with #18** (Command Queue) - This is the foundation
2. **Then #19** (Timer Callback) - Enables periodic processing
3. **Either #16** (Multiple Messages) or **#22** (Gestures) - Both are independent
4. **Then #20** (Action Sequences) - Builds on queue and timer
5. **Finally #21** (State Machine) - Most complex feature
6. **#23** (Dynamic Reconfig) and **#17** (Logging) can be done anytime

## Technical Considerations

### Command Queue Design
- Use Rust channels (mpsc) for thread-safe communication
- Consider priority levels for commands
- Implement backpressure handling

### Timer Integration
- Use safe_drive's timer API if available
- Otherwise implement with std::thread and channels
- Ensure timer doesn't block on command execution

### Message Type Abstraction
- Create trait for publishable messages
- Use enum to represent different message types
- Consider using Any trait for flexibility

### Testing Strategy
- Unit test each component in isolation
- Integration tests for command flow
- Performance tests for queue throughput
- Mock safe_drive interfaces for testing

## Success Metrics
- All tests passing
- No performance regression
- Clean API for extensions
- Well-documented code
- Example configurations for each feature