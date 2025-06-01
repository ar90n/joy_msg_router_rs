# ROS2 Parameter Server Integration

The joy_msg_router_rs node now supports dynamic reconfiguration through ROS2 parameter server integration.

## Available Parameters

### Core Parameters

- **`active_profile`** (string): Name of the currently active profile
- **`timer_rate_hz`** (double, 1.0-1000.0): Timer callback frequency in Hz
- **`global_deadzone`** (double, 0.0-1.0): Global deadzone override for all axes (-1 to use profile setting)
- **`global_scale_factor`** (double, 0.1-10.0): Global scale factor multiplier for all axes
- **`enable_button_override`** (integer, -1-15): Override enable button (-1 to use profile setting)
- **`log_level`** (string): Logging verbosity level (DEBUG, INFO, WARN, ERROR)
- **`emergency_stop`** (boolean): Emergency stop - disables all output when true

### Read-Only Parameters

- **`config_file`** (string): Path to the configuration file
- **`available_profiles`** (string array): List of available profiles in the configuration

## Usage Examples

### Using ROS2 Command Line Tools

```bash
# List all parameters
ros2 param list /joy_msg_router

# Get parameter value
ros2 param get /joy_msg_router active_profile

# Set parameter value
ros2 param set /joy_msg_router global_scale_factor 0.5

# Emergency stop
ros2 param set /joy_msg_router emergency_stop true

# Switch profile
ros2 param set /joy_msg_router active_profile drone

# Adjust timer rate
ros2 param set /joy_msg_router timer_rate_hz 100.0
```

### Parameter Validation

All parameters include validation constraints:

- **Numeric ranges**: Min/max values enforced
- **String constraints**: Valid values enumerated
- **Array limits**: Maximum length restrictions
- **Read-only protection**: Certain parameters cannot be modified

Invalid parameter changes will be rejected with descriptive error messages.

## Dynamic Reconfiguration Features

### Runtime Profile Switching

Switch between different joystick profiles without restarting the node:

```bash
# Switch to teleop profile
ros2 param set /joy_msg_router active_profile teleop

# Switch to drone profile  
ros2 param set /joy_msg_router active_profile drone
```

### Global Overrides

Override profile-specific settings globally:

```bash
# Apply 50% deadzone to all axes
ros2 param set /joy_msg_router global_deadzone 0.5

# Reduce all movements by half
ros2 param set /joy_msg_router global_scale_factor 0.5

# Override enable button to button 5
ros2 param set /joy_msg_router enable_button_override 5
```

### Emergency Control

```bash
# Emergency stop (disables all output)
ros2 param set /joy_msg_router emergency_stop true

# Resume operation
ros2 param set /joy_msg_router emergency_stop false
```

## Integration with ROS2 Tools

### Parameter Files

Save and load parameter configurations:

```bash
# Save current parameters
ros2 param dump /joy_msg_router > joy_router_params.yaml

# Load parameters from file
ros2 param load /joy_msg_router joy_router_params.yaml
```

### Launch File Integration

```xml
<launch>
  <node pkg="joy_msg_router_rs" exec="joy_msg_router" name="joy_msg_router">
    <param name="active_profile" value="drone"/>
    <param name="timer_rate_hz" value="50.0"/>
    <param name="global_scale_factor" value="0.8"/>
    <param name="log_level" value="INFO"/>
  </node>
</launch>
```

## Monitoring Parameter Changes

The node logs all parameter changes with timestamps:

```
[INFO] [timestamp] [parameter_manager]: Parameter 'active_profile' changed from 'teleop' to 'drone'
[INFO] [timestamp] [parameter_manager]: Parameter 'emergency_stop' changed from 'false' to 'true'
```

## Configuration Persistence

Parameters are applied dynamically but do not persist across node restarts unless:

1. Saved to parameter files and loaded via launch files
2. Set through rosparam server before node startup
3. Configured in launch files or configuration files

## Benefits

- **No restart required**: Change behavior without stopping the node
- **Real-time tuning**: Adjust sensitivity and timing on-the-fly
- **Emergency control**: Quick disable/enable capabilities
- **Profile switching**: Adapt to different joystick configurations
- **Debugging support**: Dynamic log level adjustment
- **ROS2 integration**: Works with standard ROS2 parameter tools

## Implementation Notes

- Parameter changes are validated before application
- Invalid changes are rejected with error messages
- Emergency stop takes priority over all other settings
- Parameter overrides are applied in addition to profile settings
- Configuration reloading updates available profiles list