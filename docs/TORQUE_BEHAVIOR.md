# G29 Force Feedback Torque Behavior

## Overview

This document explains the detailed behavior of `max_torque` and `min_torque` parameters, including how they interact with ROS2 topic commands and the G29 hardware limitations.

## Table of Contents

- [max_torque - The Safety Ceiling](#max_torque---the-safety-ceiling)
- [min_torque - The Hardware Floor](#min_torque---the-hardware-floor)
- [Practical Examples](#practical-examples)
- [Testing and Validation](#testing-and-validation)

---

## max_torque - The Safety Ceiling

### What It Does

`max_torque` is a **configurable software limit** that acts as a protective ceiling for all force commands sent to the wheel. No matter what torque value you publish via `/ff_target`, the system will never apply more than `max_torque`.

### Clamping Behavior

When you publish a torque value that exceeds `max_torque`, the system automatically clamps it:

```yaml
# Configuration: max_torque: 0.5

# Example 1: Below limit
Published: torque: 0.4
Applied:   0.4 (no clamping)
Result:    ✅ Wheel receives 0.4

# Example 2: At limit
Published: torque: 0.5
Applied:   0.5 (matches limit)
Result:    ✅ Wheel receives 0.5

# Example 3: Above limit
Published: torque: 0.6
Applied:   0.5 (clamped!)
Result:    ⚠️ Wheel receives 0.5 (not 0.6)

# Example 4: Way above limit
Published: torque: 1.0
Applied:   0.5 (clamped!)
Result:    ⚠️ Wheel receives 0.5 (not 1.0)
```

### Implementation Details

The clamping happens in the `uploadForce()` function at line 184 of `src/g29_force_feedback.cpp`:

```cpp
m_effect.u.constant.level = 0x7fff * std::min(torque, m_max_torque);
```

The `std::min()` function selects whichever value is smaller:
- If `torque < max_torque` → uses `torque`
- If `torque >= max_torque` → uses `max_torque`

### Characteristics

| Aspect | Behavior |
|--------|----------|
| **Type** | Configurable software limit |
| **Default** | 1.0 (full G29 capability = 2.5 Nm) |
| **Range** | 0.0 to 1.0 |
| **Enforcement** | Hard limit at hardware upload stage |
| **Warning** | None - silently clamps values |
| **Adjustability** | ✅ Freely adjustable based on application |

### Use Cases

#### 1. Safety Protection
```yaml
max_torque: 0.6  # Limit to 1.5 Nm to prevent injuries
```
Useful when:
- Multiple users with varying strength
- Children or elderly users
- Public demonstrations
- Shared equipment

#### 2. Comfort and Fatigue Reduction
```yaml
max_torque: 0.5  # Limit to 1.25 Nm for extended use
```
Useful when:
- Long driving sessions (> 1 hour)
- Casual gaming
- Training applications
- Reducing physical strain

#### 3. Accessibility
```yaml
max_torque: 0.3  # Limit to 0.75 Nm for users with limited strength
```
Useful when:
- Users with physical disabilities
- Rehabilitation applications
- Adaptive gaming setups

#### 4. Realistic Simulation
```yaml
max_torque: 1.0  # Full 2.5 Nm for professional racing
```
Useful when:
- Professional racing simulators
- Driver training
- Research applications
- Competition-grade setups

### Key Takeaway

**`max_torque` is YOUR control knob** - adjust it freely to match your application's safety and comfort requirements. It will never allow more force than configured, regardless of what software commands request.

---

## min_torque - The Hardware Floor

### What It Actually Is

`min_torque` is **NOT** a configurable minimum that the system enforces. Instead, it represents the **physical limitation** of the G29 hardware - the minimum torque required to actually rotate the wheel.

### The Hardware Reality

The G29 steering wheel has internal resistance from:

1. **Gear friction**: Mechanical resistance in the gear train
2. **Motor bearings**: Static friction that must be overcome to start rotation
3. **Return springs**: Mechanical springs that resist movement
4. **Belt tension**: Drive belt creates resistance
5. **Encoder friction**: Position sensing mechanism adds drag

Below approximately 0.2 (0.5 Nm), the motor **physically cannot generate enough force** to overcome these resistances.

### Critical Behavior Difference

**Unlike `max_torque`, there is NO minimum enforcement in rotate force mode:**

```cpp
// Lines 148, 152 in calcRotateForce() - No min_torque check!
torque = target.torque * m_brake_torque * -direction;  // Brake zone
torque = target.torque * direction;                    // Normal rotation
```

This means:
- You can publish `torque: 0.05` via `/ff_target`
- The system will send 0.05 to the hardware
- **But the wheel won't move** (insufficient force)

### Where min_torque IS Used

The parameter is primarily used in **auto-centering mode** as the baseline starting torque:

```cpp
// Lines 169-172 in calcCenteringForce()
double torque_range = m_auto_centering_max_torque - m_min_torque;
double power = (fabs(diff) - m_eps) / (m_auto_centering_max_position - m_eps);
double buf_torque = power * torque_range + m_min_torque;
```

In auto-centering, the force progressively increases from `min_torque` at small displacements to `auto_centering_max_torque` at large displacements.

### Characteristics

| Aspect | Behavior |
|--------|----------|
| **Type** | Hardware-derived physical limit |
| **Default** | 0.2 (0.5 Nm for G29) |
| **Range** | 0.0 to 1.0 (but < 0.2 is ineffective) |
| **Enforcement** | ❌ NOT enforced in rotate mode |
| **Warning** | None - wheel simply won't move |
| **Adjustability** | ⚠️ Not recommended - keep at 0.2 |

### Testing Results

Based on testing with G29 hardware:

```yaml
# Will NOT rotate the wheel (insufficient force)
torque: 0.05  ❌ ~0.125 Nm - Too weak
torque: 0.10  ❌ ~0.25 Nm  - Still too weak
torque: 0.15  ⚠️ ~0.375 Nm - May rotate (depends on unit condition)

# WILL rotate the wheel (sufficient force)
torque: 0.20  ✅ ~0.5 Nm   - Reliable threshold
torque: 0.25  ✅ ~0.625 Nm - Works well
torque: 0.30+ ✅ ≥0.75 Nm  - Strong rotation
```

### Factors Affecting Threshold

The actual minimum may vary based on:

| Factor | Effect on Threshold |
|--------|---------------------|
| **Wheel age/wear** | Older wheels may need higher torque |
| **Temperature** | Cold = more friction = higher threshold |
| **USB power quality** | Poor power = weaker motor = higher threshold |
| **Calibration** | Poorly calibrated = inconsistent threshold |
| **Individual unit** | Manufacturing tolerances create variation |

### Why Keep It at 0.2?

1. **Reliability**: Works consistently across different G29 units
2. **Temperature stable**: Reliable in cold or warm environments
3. **Tested default**: Package author determined this through testing
4. **Auto-centering baseline**: Provides smooth progressive centering force
5. **Safety margin**: Ensures movement actually occurs when commanded

### Experimentation Guidelines

If you want to experiment with lower values:

```yaml
# Risky - may not work reliably
min_torque: 0.15  # Only try if your specific unit seems responsive

# Acceptable range for experimentation
min_torque: 0.17-0.19  # May work, test thoroughly

# Recommended - proven reliable
min_torque: 0.20  # Keep this for production use
```

### Key Takeaway

**`min_torque` is a hardware constant, not a configuration parameter** - it represents the G29's physical capability limit. Keep it at 0.2 unless you have specific knowledge about your hardware unit's capabilities.

---

## Practical Examples

### Example 1: Safety-Limited System

```yaml
# config/g29.yaml
max_torque: 0.5
min_torque: 0.2
```

```python
# Your control code
msg.torque = 0.8  # Request 80% force

# What happens:
# 1. Your code publishes: 0.8
# 2. System clamps to max_torque: 0.8 → 0.5
# 3. Wheel receives: 0.5 (1.25 Nm)
# Result: Safe, limited force applied ✅
```

### Example 2: Weak Command

```yaml
# config/g29.yaml
max_torque: 1.0
min_torque: 0.2
```

```python
# Your control code
msg.torque = 0.1  # Request 10% force

# What happens:
# 1. Your code publishes: 0.1
# 2. System doesn't clamp (0.1 < 1.0 max)
# 3. Hardware receives: 0.1 (0.25 Nm)
# Result: No rotation - insufficient force ❌
```

### Example 3: Proper Force Command

```yaml
# config/g29.yaml
max_torque: 0.7
min_torque: 0.2
```

```python
# Your control code
msg.torque = 0.5  # Request 50% force

# What happens:
# 1. Your code publishes: 0.5
# 2. System doesn't clamp (0.5 < 0.7 max)
# 3. Hardware receives: 0.5 (1.25 Nm)
# Result: Smooth rotation with moderate force ✅
```

### Example 4: Auto-Centering Mode

```yaml
# config/g29.yaml
auto_centering: true
auto_centering_max_torque: 0.3
min_torque: 0.2
```

```python
# Wheel is displaced 10° from center (position = 0.022)

# What happens:
# 1. Small displacement → calcCenteringForce() calculates progressive force
# 2. Force starts at min_torque (0.2) and scales up based on displacement
# 3. At 10°, torque ≈ 0.21-0.22 (just above minimum)
# Result: Gentle centering force ✅

# Wheel is displaced 90° from center (position = 0.2)
# 1. Large displacement → at or beyond auto_centering_max_position
# 2. Force reaches auto_centering_max_torque (0.3)
# Result: Strong centering force ✅
```

---

## Testing and Validation

### Test 1: Verify max_torque Clamping

```bash
# Set max_torque: 0.5 in config/g29.yaml
# Restart node, then test:

ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
  "{position: 0.3, torque: 0.4}"
# Expected: Moderate force (0.4 applied)

ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
  "{position: 0.3, torque: 0.6}"
# Expected: Same force as 0.5 (clamped)

ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
  "{position: 0.3, torque: 1.0}"
# Expected: Same force as 0.5 (clamped)
```

**Validation**: All commands with torque ≥ 0.5 should feel identical.

### Test 2: Verify min_torque Threshold

```bash
# Keep default min_torque: 0.2 in config/g29.yaml

ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
  "{position: 0.3, torque: 0.1}"
# Expected: No wheel movement ❌

ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
  "{position: 0.3, torque: 0.15}"
# Expected: No or minimal wheel movement ⚠️

ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
  "{position: 0.3, torque: 0.2}"
# Expected: Wheel starts rotating ✅

ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
  "{position: 0.3, torque: 0.3}"
# Expected: Clear, smooth rotation ✅
```

**Validation**: Noticeable difference between 0.1 (no movement) and 0.2+ (movement).

### Test 3: Progressive Torque Response

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_g29_force_feedback.msg import ForceFeedback
import time

class TorqueTest(Node):
    def __init__(self):
        super().__init__('torque_test')
        self.publisher = self.create_publisher(ForceFeedback, '/ff_target', 10)
    
    def run_test(self):
        """Test progressive torque from 0.1 to 1.0"""
        for torque in [0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
            msg = ForceFeedback()
            msg.position = 0.3
            msg.torque = torque
            self.publisher.publish(msg)
            print(f"Testing torque: {torque}")
            time.sleep(3)  # Feel each level for 3 seconds
        
        # Return to center
        msg = ForceFeedback()
        msg.position = 0.0
        msg.torque = 0.5
        self.publisher.publish(msg)

def main():
    rclpy.init()
    tester = TorqueTest()
    tester.run_test()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Results**:
- 0.1-0.15: Little to no movement
- 0.2: Movement starts
- 0.3+: Progressively stronger force
- With `max_torque: 0.5`, values above 0.5 feel identical

---

## Summary

| Parameter | Nature | Your Control | Hardware Constraint |
|-----------|--------|--------------|---------------------|
| **max_torque** | Software limit | ✅ Full control | ❌ No constraint |
| **min_torque** | Physical limit | ⚠️ Limited | ✅ Hardware bound |

### Key Principles

1. **max_torque = Your Safety Ceiling**
   - Adjust freely for your application
   - Protects users and hardware
   - Silently clamps excessive commands

2. **min_torque = Hardware's Physical Floor**
   - Keep at default (0.2) for reliability
   - Below this, wheel won't rotate
   - Used for auto-centering baseline

3. **Design Your System Around These Limits**
   - Always publish `torque >= min_torque` (0.2)
   - Set `max_torque` based on safety requirements
   - Test with your specific hardware unit
   - Document your configuration choices

### Quick Reference

```yaml
# Conservative setup (safe, gentle)
max_torque: 0.5
min_torque: 0.2

# Balanced setup (default)
max_torque: 1.0
min_torque: 0.2

# Never recommended
max_torque: 1.0
min_torque: 0.1  # ❌ Below hardware capability
```
