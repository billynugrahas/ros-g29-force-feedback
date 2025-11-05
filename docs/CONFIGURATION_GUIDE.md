# G29 Force Feedback Configuration Guide

## Overview

This package provides ROS2-based force feedback control for the Logitech G29 steering wheel. It allows you to control the wheel's rotation and resistance through ROS2 topics, making it ideal for driving simulators and autonomous vehicle interfaces.

## Configuration Parameters

All parameters are defined in [`config/g29.yaml`](../config/g29.yaml) and loaded by the `g29_force_feedback` node.

### Device Configuration

#### `device_name` (string)
- **Default**: `/dev/input/event25`
- **Description**: The Linux input event device path for the G29 wheel
- **How to find**: Run `cat /proc/bus/input/devices | grep -iA4 logitech` and look for the event handler
- **Note**: This number can change after reboot, so verify before each use

#### `loop_rate` (double, seconds)
- **Default**: `0.1` (100ms)
- **Range**: > 0.0
- **Description**: Control loop frequency for reading wheel position and uploading force commands
- **Formula**: Loop executes every `loop_rate * 1000` milliseconds
- **Trade-offs**: 
  - Lower values = more responsive but higher CPU usage
  - Higher values = less responsive but lower CPU usage

### Torque Parameters

#### `max_torque` (double, normalized)
- **Default**: `1.0`
- **Range**: `min_torque` < `max_torque` ≤ `1.0`
- **Description**: Maximum torque applied to the wheel - acts as a configurable safety limit
- **Physical mapping**: For G29, `1.0 = 2.5 Nm`
- **Behavior**: When you publish a torque value exceeding `max_torque`, it automatically clamps to `max_torque`
- **Nature**: Software-enforced ceiling (freely adjustable)

**Quick Example:**
```yaml
# If max_torque: 0.5 in config
# Publishing torque: 0.6 → Wheel receives 0.5 (clamped)
# Publishing torque: 0.4 → Wheel receives 0.4 (no clamping)
```

**📖 For detailed behavior, clamping mechanics, use cases, and testing examples, see [TORQUE_BEHAVIOR.md](TORQUE_BEHAVIOR.md#max_torque---the-safety-ceiling)**

#### `min_torque` (double, normalized)
- **Default**: `0.2`
- **Range**: > 0.0, < `max_torque`
- **Description**: Minimum effective torque threshold representing G29 hardware limitations
- **Physical mapping**: For G29, `0.2 = 0.5 Nm`
- **Behavior**: Values below 0.2 typically cannot overcome internal friction - wheel won't rotate
- **Nature**: Hardware-derived constant (not recommended to change)

**Critical Note:**
Unlike `max_torque`, this is **NOT enforced** in rotate mode. Publishing `torque: 0.1` sends the command to hardware, but the wheel won't physically move due to insufficient force.

**Primary Usage:** Baseline torque for auto-centering force calculations

**📖 For detailed hardware limitations, testing results, and why 0.2 is the recommended value, see [TORQUE_BEHAVIOR.md](TORQUE_BEHAVIOR.md#min_torque---the-hardware-floor)**

### Braking System

The braking system provides smooth deceleration as the wheel approaches the target position, preventing oscillation.

#### `brake_torque` (double, normalized)
- **Default**: `0.2`
- **Range**: 0.0 to 1.0
- **Description**: Torque multiplier applied when the wheel enters the brake zone
- **Formula**: `actual_brake_torque = target.torque × brake_torque × -direction`
- **Effect**: Creates resistance to slow down the wheel before reaching the target

#### `brake_position` (double, normalized)
- **Default**: `0.1`
- **Range**: 0.0 to 1.0
- **Description**: Position threshold that activates braking
- **Formula**: Brake zone = `target_position ± brake_position × max_angle`
- **Example**: With G29 (max_angle=450°), brake_position=0.1 means braking starts 45° before target

**Braking Behavior** (from `calcRotateForce()`):
1. **Far from target** (`|diff| ≥ brake_position`): Apply full torque in direction of target
2. **In brake zone** (`eps < |diff| < brake_position`): Apply reverse torque to slow down
3. **At target** (`|diff| < eps`): Zero torque, position reached

### Auto-Centering Mode

When enabled, the wheel automatically returns to the center position (0.0) with progressive force.

#### `auto_centering` (boolean)
- **Default**: `false`
- **Description**: Enable/disable auto-centering mode
- **When enabled**: Wheel automatically centers regardless of ROS topic commands
- **When disabled**: Wheel follows ROS topic commands with braking behavior

#### `auto_centering_max_torque` (double, normalized)
- **Default**: `0.3`
- **Range**: `min_torque` to `max_torque`
- **Description**: Maximum torque applied during auto-centering
- **Usage**: Upper limit of centering force at maximum displacement

#### `auto_centering_max_position` (double, normalized)
- **Default**: `0.2`
- **Range**: 0.0 to 1.0
- **Description**: Position at which maximum centering torque is reached
- **Formula**: Max torque position = `± auto_centering_max_position × max_angle`

**Auto-Centering Force Calculation** (from `calcCenteringForce()`):
```
torque_range = auto_centering_max_torque - min_torque
power = (|position_diff| - eps) / (auto_centering_max_position - eps)
torque = min(power × torque_range + min_torque, auto_centering_max_torque)
```

This creates a progressive spring-like force:
- At center (`|diff| < eps`): No force
- Small displacement: Gradually increasing from `min_torque`
- Large displacement (`≥ auto_centering_max_position`): Full `auto_centering_max_torque`

### Precision Control

#### `eps` (double, normalized)
- **Default**: `0.05`
- **Range**: > 0.0, typically 0.01 to 0.1
- **Description**: Dead zone threshold around target position
- **Formula**: Dead zone = `target_position ± eps`
- **Effect**: Wheel within this range is considered "at target" and receives zero torque
- **Purpose**: Prevents oscillation and hunting around the target position

## ROS2 Topic Interface

### Subscribed Topics

#### `/ff_target` (`ros_g29_force_feedback/msg/ForceFeedback`)

**Message Definition:**
```
std_msgs/Header header
float32 position  # Target wheel position (normalized)
float32 torque    # Desired torque magnitude (normalized)
```

**Message Fields:**

- **`position`** (float32, normalized range: -1.0 to 1.0)
  - Target wheel angle normalized by maximum rotation
  - **G29 mapping**: ±1.0 = ±450° (900° total rotation)
  - **Sign convention**: 
    - Positive = clockwise rotation
    - Negative = counterclockwise rotation
  - **Example**: `position = 0.5` → rotate to 225° clockwise from center

- **`torque`** (float32, normalized range: 0.0 to 1.0)
  - Desired force magnitude (absolute value used internally)
  - Clamped to `max_torque` parameter
  - Direction determined automatically based on difference from current position
  - **Example**: `torque = 0.8` → 80% of maximum torque (2.0 Nm for G29)

**Publishing Example:**
```bash
ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, \
    position: 0.3, torque: 0.5}"
```

This command rotates the wheel to 135° clockwise (0.3 × 450°) with 50% torque (1.25 Nm).

**Behavior Notes:**
- Publishing rate is **not restricted** - node responds to every message
- Same position+torque messages are ignored to prevent redundant processing
- Torque is always treated as absolute value (negative signs ignored)
- In `auto_centering` mode, topic messages are received but centering force overrides them

## System Architecture

### Control Flow

```
ROS Topic /ff_target → targetCallback
                           ↓
                    Update m_target
                           ↓
Timer Loop (loop_rate) → Read Wheel Position
                           ↓
                    ┌──────┴──────┐
                    ↓             ↓
         Auto Centering?    Brake Range?
                    ↓             ↓
         calcCenteringForce   calcRotateForce
                    ↓             ↓
                    └──────┬──────┘
                           ↓
                    uploadForce
                           ↓
                  Linux FF API
                           ↓
                    G29 Hardware
```

### Force Calculation Modes

The node operates in one of two modes per control loop:

1. **Rotate Force Mode** (default when `auto_centering=false`)
   - Actively drives wheel to target position
   - Three-phase approach: accelerate → brake → stop
   - Controlled by `calcRotateForce()`

2. **Centering Force Mode** (when `auto_centering=true` OR in brake range)
   - Progressive spring-like force toward target
   - Force proportional to distance from target
   - Controlled by `calcCenteringForce()`

### Hardware Interface

The node communicates with the G29 through the [Linux Force Feedback API](https://www.kernel.org/doc/html/v5.4/input/ff.html):

1. **Initialization** (`initDevice()`):
   - Opens device at `/dev/input/eventX`
   - Verifies FF_CONSTANT support
   - Disables hardware auto-centering
   - Creates persistent force effect

2. **Position Reading** (`loop()`):
   - Reads `EV_ABS` events from device
   - Normalizes raw axis values to [-1.0, 1.0] range

3. **Force Upload** (`uploadForce()`):
   - Updates FF_CONSTANT effect parameters
   - Uses `ioctl(EVIOCSFF)` to upload to kernel
   - Effect runs continuously (replay.length = 0xffff)

## Practical Usage Examples

### Example 1: Basic Position Command
```bash
# Rotate wheel 90° clockwise with 60% torque
ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
  "{position: 0.2, torque: 0.6}"
```

### Example 2: CARLA Simulator Integration
See [`examples/carla_control.py`](../examples/carla_control.py) for a complete example that:
- Reads steering angle from CARLA vehicle
- Publishes position commands at 10Hz
- Creates realistic force feedback matching vehicle steering

```python
steering_angle = actor.get_control().steer  # CARLA normalized steering
msg.position = steering_angle               # Direct mapping
msg.torque = 0.8                            # Constant resistance
publisher.publish(msg)
```

### Example 3: Smooth Trajectory Following
```python
# Slowly sweep from -45° to +45°
import numpy as np
for angle in np.linspace(-0.1, 0.1, 100):
    msg = ForceFeedback()
    msg.position = angle
    msg.torque = 0.5
    publisher.publish(msg)
    time.sleep(0.05)
```

## Tuning Guide

### Problem: Wheel oscillates around target

**Solutions:**
- Increase `eps` (expand dead zone)
- Increase `brake_position` (start braking earlier)
- Increase `brake_torque` (stronger braking)
- Decrease `max_torque` (reduce overall force)

### Problem: Wheel moves too slowly

**Solutions:**
- Increase `max_torque`
- Decrease `brake_position` (brake later)
- Ensure `torque` in ROS message is > `min_torque`

### Problem: Jerky or unresponsive

**Solutions:**
- Decrease `loop_rate` (faster updates)
- Ensure publishing rate matches application needs
- Check for message processing delays

### Problem: Auto-centering too weak/strong

**Solutions:**
- Adjust `auto_centering_max_torque`
- Modify `auto_centering_max_position` to change force curve
- Tune `min_torque` for baseline centering force

## Technical Specifications

### G29 Hardware Limits
- **Maximum rotation**: ±450° (900° total)
- **Maximum torque**: 2.5 Nm (at normalized value 1.0)
- **Force feedback type**: FF_CONSTANT (constant force effects)
- **Position resolution**: Determined by Linux input subsystem (typically 16-bit)

### Node Performance
- **Default loop rate**: 10 Hz (100ms period)
- **Message latency**: Near real-time (depends on system load)
- **Resource usage**: Minimal CPU, no GPU required

### Dependencies
- ROS2 (Humble or later recommended)
- Linux kernel with `CONFIG_LOGIWHEELS_FF=y`
- Standard C++ libraries (linux/input.h for force feedback)

## Launch Configuration

Use the provided launch file to start the node with parameters:

```bash
ros2 launch ros_g29_force_feedback g29_feedback.launch.py
```

The launch file automatically loads `config/g29.yaml` parameters and supports namespace configuration for multi-device setups.

## Parameter Reference Table

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `device_name` | string | `/dev/input/event25` | Valid device path | Linux input event device for G29 |
| `loop_rate` | double | `0.1` | > 0.0 | Control loop period in seconds |
| `max_torque` | double | `1.0` | 0.0 to 1.0 | Maximum normalized torque (1.0 = 2.5 Nm) |
| `min_torque` | double | `0.2` | 0.0 to `max_torque` | Minimum effective torque |
| `brake_torque` | double | `0.2` | 0.0 to 1.0 | Braking torque multiplier |
| `brake_position` | double | `0.1` | 0.0 to 1.0 | Normalized brake zone threshold |
| `auto_centering_max_torque` | double | `0.3` | `min_torque` to `max_torque` | Max auto-centering torque |
| `auto_centering_max_position` | double | `0.2` | 0.0 to 1.0 | Position for max centering force |
| `eps` | double | `0.05` | > 0.0 | Dead zone threshold |
| `auto_centering` | bool | `false` | true/false | Enable auto-centering mode |

## Summary

This package provides precise force feedback control through:
- **Parameters**: 10 tunable configuration values for behavior customization
- **ROS2 Topic**: Single topic (`/ff_target`) with position and torque commands
- **Two Modes**: Active positioning or auto-centering
- **Smart Braking**: Automatic deceleration prevents oscillation
- **Real-time Control**: Low-latency force updates via Linux kernel API

The system is designed for intuitive use while providing advanced tuning capabilities for specialized applications.