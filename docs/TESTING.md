# Joy Message Router Tests

This directory contains node-level tests for the joy_msg_router_rs package.

## Test Files

- `test_simple.py` - Basic functionality tests that can be run standalone
- `test_integration.py` - Integration tests using launch_testing framework
- `test_joy_router.py` - Comprehensive node-level tests using pytest
- `test_service_calls.py` - Tests for service call functionality

## Running Tests

### Unit Tests (Rust)
```bash
# From package directory
cd ~/ros2_ws/src/joy_msg_router_rs
cargo test
cargo test -- --nocapture  # To see output
```

### Simple Tests (Python - Quick testing)
```bash
# From workspace root
source activate_build_env
source install/setup.bash
python3 src/joy_msg_router_rs/tests/test_simple.py
```

### Integration Tests with pytest
```bash
# From workspace root
source activate_build_env
source install/setup.bash
pytest src/joy_msg_router_rs/tests/test_joy_router.py -v
```

### Launch Testing (ROS2 standard)
```bash
# Run via colcon test
colcon test --packages-select joy_msg_router_rs
colcon test-result --verbose
```

## Test Coverage

The tests verify:
1. **Node Launch** - The node starts successfully and creates expected topics
2. **Basic Message Routing** - Joy messages are converted to various message types correctly
3. **Deadzone Filtering** - Values below deadzone threshold are filtered out
4. **Enable Button** - Movement is disabled when enable button is not pressed
5. **Button Actions** - Button presses trigger configured actions
6. **Service Calls** - Buttons can trigger service calls
7. **Configuration Loading** - Node loads hierarchical YAML and flat parameter files correctly
8. **Modifier System** - Dynamic parameter modification works correctly
9. **Profile Selection** - Different profiles can be loaded from config files

## Writing New Tests

When adding new tests:
1. Use the existing test structure as a template
2. Ensure proper cleanup of ROS2 resources
3. Use appropriate timeouts for message passing
4. Test both positive and negative cases
5. Document what each test verifies