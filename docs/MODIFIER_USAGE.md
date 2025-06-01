# Modifier Action Type Usage Guide

The modifier action type allows dynamic modification of other input mappings' scale, offset, and deadzone values based on button presses or axis positions.

## Basic Concept

Modifiers work in a two-pass system:
1. **First pass**: Collect all active modifiers
2. **Second pass**: Apply modifiers to regular mappings before processing

## Configuration

### Modifier Action Fields

```yaml
action:
  type: modifier
  targets: []              # List of targets to modify (required)
  scale_multiplier: 2.0    # Multiply target's scale (optional)
  offset_delta: 0.1        # Add to target's offset (optional)
  deadzone_override: 0.05  # Override target's deadzone (optional)
  apply_gradually: false   # For axis modifiers - interpolate by axis value
```

### Target Specification

Targets can be specified in two ways:

#### 1. By Mapping ID
```yaml
targets:
  - mapping_id: "forward_movement"
```
Requires the target mapping to have an `id` field.

#### 2. By Source
```yaml
targets:
  - source:
      type: axis    # or "button"
      index: 1      # source index
```
Modifies all mappings with the specified source.

## Examples

### Turbo Button
```yaml
# Double speed when R1 is held
- source_type: button
  source_index: 5
  action:
    type: modifier
    targets:
      - mapping_id: "movement"
    scale_multiplier: 2.0
```

### Precision Mode
```yaml
# Reduce speed and deadzone for fine control
- source_type: button
  source_index: 4
  action:
    type: modifier
    targets:
      - mapping_id: "movement"
    scale_multiplier: 0.3
    deadzone_override: 0.02
```

### Variable Speed Control
```yaml
# Use trigger for gradual speed control
- source_type: axis
  source_index: 5
  action:
    type: modifier
    targets:
      - source:
          type: axis
          index: 1
    scale_multiplier: 3.0    # Max 3x at full trigger
    apply_gradually: true    # Scale based on trigger value
```

## Modifier Stacking

Multiple modifiers can affect the same mapping:
- **Scale**: Multiplicative (scale1 × scale2 × ...)
- **Offset**: Additive (offset1 + offset2 + ...)
- **Deadzone**: Minimum value (most restrictive)

Example with two modifiers active:
- Base scale: 1.0
- Modifier 1: scale_multiplier = 2.0
- Modifier 2: scale_multiplier = 0.5
- Result: 1.0 × 2.0 × 0.5 = 1.0

## ROS2 Parameter Format

For ROS2 parameters (flat format), modifiers use indexed notation:

```yaml
joy_msg_router:
  ros__parameters:
    # Regular mapping
    input_mappings.0.id: "movement"
    input_mappings.0.source_type: "axis"
    input_mappings.0.source_index: 1
    input_mappings.0.scale: 0.5
    input_mappings.0.action_type: "publish"
    input_mappings.0.topic: "/cmd_vel"
    input_mappings.0.message_type: "geometry_msgs/msg/Twist"
    input_mappings.0.field: "linear.x"
    
    # Modifier
    input_mappings.1.source_type: "button"
    input_mappings.1.source_index: 5
    input_mappings.1.action_type: "modifier"
    input_mappings.1.targets.0.mapping_id: "movement"
    input_mappings.1.scale_multiplier: 2.0
```

## Gradual Modifiers

When `apply_gradually: true` is set for an axis modifier:
- The modification strength is proportional to the axis value
- At axis value 0.0: No modification
- At axis value 1.0: Full modification
- Example: With scale_multiplier: 3.0 and axis at 0.5
  - Effective multiplier = 1.0 + (3.0 - 1.0) × 0.5 = 2.0

## Use Cases

1. **Turbo/Boost Mode**: Button increases maximum speed
2. **Precision Mode**: Button reduces sensitivity for fine control
3. **Safety Limiter**: Reduce speed based on proximity sensors
4. **Variable Sensitivity**: Adjust control sensitivity on the fly
5. **Emergency Stop**: Zero out all movement instantly
6. **Adaptive Control**: Modify behavior based on system state