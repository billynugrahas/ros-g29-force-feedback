# G29 Auto-Centering System Behavior

## Table of Contents

- [Overview](#overview)
- [System Parameters](#system-parameters)
- [How Auto-Centering Works](#how-auto-centering-works)
- [Mode Comparison](#mode-comparison)
- [Parameter Deep Dive](#parameter-deep-dive)
- [Configuration Examples](#configuration-examples)
- [Testing Guide](#testing-guide)
- [Use Case Recommendations](#use-case-recommendations)
- [Common Questions](#common-questions)

---

## Overview

The auto-centering system provides progressive spring-like force that pulls the wheel toward a target position. It can operate in two modes:

1. **Auto-Centering Mode** (`auto_centering: true`) - Always applies centering force
2. **Manual Control Mode** (`auto_centering: false`) - Uses centering only during braking phase

The system uses the same centering force calculation ([`calcCenteringForce()`](../src/g29_force_feedback.cpp:158)) for both modes, but triggered by different conditions.

---

## System Parameters

### `auto_centering` (boolean)
- **Type**: `bool`
- **Default**: `false`
- **Range**: `true` or `false`
- **Description**: Enable/disable auto-centering mode

**What it controls**: Whether the wheel constantly applies centering force or responds to ROS topic commands.

---

### `auto_centering_max_torque` (double, normalized)
- **Type**: `double` (normalized)
- **Default**: `0.3`
- **Range**: `min_torque` to `max_torque` (typically `0.2` to `1.0`)
- **Description**: Maximum centering force at full displacement
- **Physical mapping**: For G29, `1.0 = 2.5 Nm`

**What it controls**: How strong the centering force is at maximum displacement.

---

### `auto_centering_max_position` (double, normalized)
- **Type**: `double` (normalized)
- **Default**: `0.2`
- **Range**: `0.0` to `1.0`
- **Description**: Distance at which maximum centering torque is reached
- **Formula**: `max_torque_distance = auto_centering_max_position × 450°` (for G29)

**What it controls**: How far you must turn before reaching maximum centering force (controls force curve steepness).

---

## How Auto-Centering Works

### The Force Calculation

From [`src/g29_force_feedback.cpp:158-174`](../src/g29_force_feedback.cpp:158-174):

```cpp
void G29ForceFeedback::calcCenteringForce(double &torque,
                                          const ros_g29_force_feedback::msg::ForceFeedback &target,
                                          const double &current_position) {
    
    double diff = target.position - current_position;
    double direction = (diff > 0.0) ? 1.0 : -1.0;
    
    if (fabs(diff) < m_eps)
        torque = 0.0;  // Dead zone
    
    else {
        double torque_range = m_auto_centering_max_torque - m_min_torque;
        double power = (fabs(diff) - m_eps) / (m_auto_centering_max_position - m_eps);
        double buf_torque = power * torque_range + m_min_torque;
        torque = std::min(buf_torque, m_auto_centering_max_torque) * direction;
    }
}
```

### The Formula Breakdown

```
torque_range = auto_centering_max_torque - min_torque
power = (distance_from_target - eps) / (auto_centering_max_position - eps)
torque = min(power × torque_range + min_torque, auto_centering_max_torque)
```

**Key points**:
1. **`power`** is a ratio (0.0 to 1.0+) determining force percentage
2. At `distance = eps`: `power = 0` → Apply `min_torque` (0.2)
3. At `distance = auto_centering_max_position`: `power = 1.0` → Apply `auto_centering_max_torque`
4. Beyond `auto_centering_max_position`: Clamped to `auto_centering_max_torque`

---

### When Is Centering Force Applied?

From [`src/g29_force_feedback.cpp:121-128`](../src/g29_force_feedback.cpp:121-128):

```cpp
if (m_is_brake_range || m_auto_centering) {
    calcCenteringForce(m_torque, m_target, m_position);
    m_attack_length = 0.0;
    
} else {
    calcRotateForce(m_torque, m_attack_length, m_target, m_position);
    m_is_target_updated = false;
}
```

**Two conditions trigger centering force**:
1. **`m_auto_centering == true`** → Always uses centering force
2. **`m_is_brake_range == true`** → Entered brake zone (even with `auto_centering: false`)

---

## Mode Comparison

### Mode 1: `auto_centering: true` (Auto-Centering Mode)

**Behavior**:
- Wheel **constantly** pulls toward `target.position` (defaults to 0° if no ROS messages)
- Progressive spring-like force everywhere
- No acceleration phase, only smooth centering
- Works **without any ROS messages**

**Code path**:
```
Always: calcCenteringForce(m_target, m_position)
```

**Force profile**:
```
Force
0.3 |────────────┐
    |            │
    |            │  Progressive
0.2 |────────────┤  spring force
    |            │  everywhere
    |____________│___________________
    -450°      -90°  0°  +90°    +450°
```

**Parameters that matter**:
- ✅ `auto_centering_max_torque` - Controls max force
- ✅ `auto_centering_max_position` - Controls force curve
- ✅ `min_torque` - Baseline force
- ✅ `eps` - Dead zone
- ❌ `brake_torque` - **IGNORED**
- ❌ `brake_position` - **IGNORED**

---

### Mode 2: `auto_centering: false` (Manual Control Mode)

**Behavior**:
- Responds to ROS topic commands
- Uses **3-phase control**:
  1. **Acceleration** (far from target) - Full speed toward target
  2. **Braking/Centering** (near target) - Deceleration with centering force
  3. **Dead zone** (at target) - Zero force
- Requires ROS messages to know target position

**Code path**:
```
Far from target:  calcRotateForce() - acceleration
Near target:      calcCenteringForce() - braking (brake zone)
At target:        Zero force
```

**Force profile**:
```
Force
0.8 |────────────┐
    |            │  Acceleration
    |            │  (calcRotateForce)
    |            └──┐
0.3 |               │ Centering/brake
0.2 |               │ (calcCenteringForce)
    |_______________└────────────────
    -450°        -45°  -9°    0°
                  ↑    ↑
           brake zone dead zone
```

**Parameters that matter**:
- **Far from target (acceleration)**:
  - ✅ `brake_position` - When to start braking
  - ✅ `brake_torque` - Brake strength
  - ✅ `eps` - Dead zone
- **Near target (braking/centering)**:
  - ✅ `auto_centering_max_torque` - Max brake force
  - ✅ `auto_centering_max_position` - Brake force curve
  - ✅ `min_torque` - Baseline force
  - ✅ `eps` - Dead zone

---

## Parameter Deep Dive

### `auto_centering_max_torque` - Maximum Centering Force

**Meaning**: The strongest force the centering system can apply when you're far from target.

**⚠️ IMPORTANT**: This parameter affects **BOTH** modes:
- **`auto_centering: true`**: Controls centering force everywhere
- **`auto_centering: false`**: Controls braking force in brake zone (near target)

Even with `auto_centering: false`, increasing this value will make braking stronger!

#### Effect of Changing Values

| Value | Force (Nm) | Feel | Use Case |
|-------|-----------|------|----------|
| **0.2** | 0.5 Nm | Very light | Minimal centering, comfortable |
| **0.3** (default) | 0.75 Nm | Light-moderate | Balanced, good default |
| **0.5** | 1.25 Nm | Moderate | Noticeable centering |
| **0.8** | 2.0 Nm | Strong | Firm centering preference |
| **1.0** | 2.5 Nm | Very strong | Maximum centering, may be tiring |

#### Example Calculation

**Config**: `auto_centering_max_torque: 0.3`, `auto_centering_max_position: 0.2`, `min_torque: 0.2`

**Wheel at 90° from target** (position = 0.2):
```
power = (0.2 - 0.02) / (0.2 - 0.02) = 1.0
torque = 1.0 × (0.3 - 0.2) + 0.2 = 0.3

Result: Apply maximum force (0.3)
```

**Increasing to 0.8**:
```
power = 1.0 (same)
torque = 1.0 × (0.8 - 0.2) + 0.2 = 0.8

Result: Much stronger pull (0.8)
```

---

### `auto_centering_max_position` - Force Curve Steepness

**Meaning**: How far you must turn before the centering force reaches its maximum.

#### Effect of Changing Values

| Value | Distance (degrees) | Force Curve | Feel | Use Case |
|-------|-------------------|-------------|------|----------|
| **0.05** | 22.5° | Very steep | Aggressive, immediate strong force | Too harsh (not recommended) |
| **0.1** | 45° | Steep | Firm, quick ramp-up | Strong centering preference |
| **0.2** (default) | 90° | Moderate | Balanced progression | Good default |
| **0.4** | 180° | Gradual | Gentle, slow ramp-up | Comfort priority |
| **0.6** | 270° | Very gradual | Very soft, barely noticeable | Minimal resistance |

#### Visual Force Curves

**Steep curve** (`auto_centering_max_position: 0.1`):
```
Force
0.3 |      ┌────────────────────────
    |     /
    |    /   Reaches max at 45°
0.2 |___/    (aggressive)
    |________________________
    0°  45°  90°    180°   Position
```

**Moderate curve** (`auto_centering_max_position: 0.2` - default):
```
Force
0.3 |           ┌─────────────────
    |          /
    |         /  Reaches max at 90°
0.2 |________/   (balanced)
    |________________________
    0°     90°     180°   Position
```

**Gentle curve** (`auto_centering_max_position: 0.4`):
```
Force
0.3 |                   ┌─────────
    |                  /
    |                 /  Reaches max at 180°
0.2 |________________/   (comfortable)
    |________________________
    0°     90°     180°    Position
```

#### Example Calculation

**Config**: `auto_centering_max_position: 0.2`, `auto_centering_max_torque: 0.3`, `min_torque: 0.2`

**Wheel at 45° from target** (position = 0.1):
```
power = (0.1 - 0.02) / (0.2 - 0.02) = 0.444
torque = 0.444 × (0.3 - 0.2) + 0.2 = 0.244

Result: 44% of the way to max force
```

**With `auto_centering_max_position: 0.1` (steeper)**:
```
power = (0.1 - 0.02) / (0.1 - 0.02) = 1.0
torque = 1.0 × (0.3 - 0.2) + 0.2 = 0.3

Result: Already at maximum force!
```

**With `auto_centering_max_position: 0.4` (gentler)**:
```
power = (0.1 - 0.02) / (0.4 - 0.02) = 0.211
torque = 0.211 × (0.3 - 0.2) + 0.2 = 0.221

Result: Only 21% of the way to max force
```

---

## Configuration Examples

### Example 1: Default Balanced Setup

```yaml
auto_centering: false
auto_centering_max_torque: 0.3
auto_centering_max_position: 0.2
```

**Behavior**:
- Used in brake zone when `auto_centering: false`
- Moderate force curve
- Reaches max at 90° displacement
- Good for most applications

**Use case**: General-purpose driving simulation

---

### Example 2: Strong Auto-Centering

```yaml
auto_centering: true
auto_centering_max_torque: 0.8
auto_centering_max_position: 0.15
```

**Behavior**:
- Always centering to 0° (or last ROS target)
- Strong maximum force (2.0 Nm)
- Steep curve (max at 67.5°)
- Wheel wants to return to center firmly

**Use case**: Hardware testing, safety mode, realistic spring-return feel

---

### Example 3: Gentle Comfort Setup

```yaml
auto_centering: true
auto_centering_max_torque: 0.3
auto_centering_max_position: 0.4
```

**Behavior**:
- Always centering
- Moderate maximum force
- Very gradual curve (max at 180°)
- Comfortable, minimal resistance

**Use case**: Casual gaming, long sessions, accessibility

---

### Example 4: Precise Control (Recommended)

```yaml
auto_centering: false
brake_torque: 1.0
brake_position: 0.1
eps: 0.02
auto_centering_max_torque: 0.3
auto_centering_max_position: 0.2
```

**Behavior**:
- ROS topic control
- Strong braking when approaching target
- Centering force in brake zone
- High precision (±9° dead zone)

**Use case**: CARLA simulator, autonomous vehicle control, robotics

---

## Testing Guide

### Test 1: Baseline Configuration

**Config**:
```yaml
auto_centering: true
auto_centering_max_torque: 0.3
auto_centering_max_position: 0.2
```

**Steps**:
1. Edit `config/g29.yaml` with above values
2. Restart node: `ros2 launch ros_g29_force_feedback g29_feedback.launch.py`
3. Don't publish any ROS messages
4. Turn wheel manually:
   - **At 0°**: No force (dead zone)
   - **At 45°**: Moderate resistance
   - **At 90°**: Strong resistance (max force)
   - **At 180°**: Same as 90° (clamped)

**Expected**: Force gradually increases 0° → 90°, then stays constant

---

### Test 2: Steep Curve (Aggressive)

**Config**:
```yaml
auto_centering: true
auto_centering_max_torque: 0.3
auto_centering_max_position: 0.1  # Changed
```

**Steps**:
1. Change `auto_centering_max_position: 0.1`
2. Restart node
3. Turn wheel:
   - **At 45°**: **Strong resistance** (already at max!)
   - **At 90°**: Same as 45°

**Expected**: Force ramps up **very quickly**, aggressive feel

---

### Test 3: Gentle Curve (Comfortable)

**Config**:
```yaml
auto_centering: true
auto_centering_max_torque: 0.3
auto_centering_max_position: 0.4  # Changed
```

**Steps**:
1. Change `auto_centering_max_position: 0.4`
2. Restart node
3. Turn wheel:
   - **At 45°**: Weak resistance (~25% of max)
   - **At 90°**: Moderate resistance (~50% of max)
   - **At 180°**: Finally strong (reaches max)

**Expected**: Force ramps up **slowly**, gentle feel

---

### Test 4: Strong Maximum Force

**Config**:
```yaml
auto_centering: true
auto_centering_max_torque: 0.8  # Changed
auto_centering_max_position: 0.1
```

**Steps**:
1. Change both parameters
2. Restart node
3. Turn wheel to 45°

**Expected**: **Very strong** pull to center, like a stiff spring

---

### Test 5: Manual Control Mode

**Config**:
```yaml
auto_centering: false
```

**Steps**:
1. Set `auto_centering: false`
2. Restart node
3. Publish ROS message:
   ```bash
   ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
     "{position: 0.3, torque: 0.8}"
   ```
4. Observe:
   - Wheel accelerates toward 135° (0.3 × 450°)
   - Enters brake zone at ~45° from target
   - Uses centering force to smoothly stop

**Expected**: 3-phase motion (acceleration → brake → stop)

---

## Use Case Recommendations

### Driving Simulators (CARLA, Racing Games)

**Recommended**:
```yaml
auto_centering: false      # Essential for dynamic control
brake_torque: 0.8          # Strong, realistic braking
brake_position: 0.1
eps: 0.02                  # Precise positioning
auto_centering_max_torque: 0.3
auto_centering_max_position: 0.2
```

**Why**: Provides precise tracking of vehicle steering with realistic brake behavior

---

### Autonomous Vehicle Interface

**Recommended**:
```yaml
auto_centering: false
brake_torque: 1.0          # Maximum precision
brake_position: 0.1
eps: 0.02
auto_centering_max_torque: 0.3
auto_centering_max_position: 0.2
```

**Why**: High precision, responsive to commands, minimal drift

---

### Hardware Testing / Demonstrations

**Recommended**:
```yaml
auto_centering: true       # No ROS messages needed
auto_centering_max_torque: 0.4
auto_centering_max_position: 0.2
```

**Why**: Works standalone, always returns to center, easy to demonstrate

---

### Safety / Fallback Mode

**Recommended**:
```yaml
auto_centering: true       # Always returns to safe position
auto_centering_max_torque: 0.5
auto_centering_max_position: 0.15
```

**Why**: Ensures wheel returns to center if control is lost

---

### Comfortable Casual Gaming

**Recommended**:
```yaml
auto_centering: false
brake_torque: 0.4          # Gentle
brake_position: 0.15       # Early braking
eps: 0.03                  # Comfortable tolerance
auto_centering_max_torque: 0.3
auto_centering_max_position: 0.3
```

**Why**: Smooth, comfortable feel for extended play sessions

---

## Common Questions

### Q1: What is the difference between `auto_centering: true` and `false`?

**Answer**:

| Aspect | `true` | `false` |
|--------|--------|---------|
| **Force application** | Constant centering everywhere | Only near target (brake zone) |
| **Requires ROS messages** | No | Yes |
| **Target position** | 0° or last ROS command | Any position via ROS |
| **Force profile** | Progressive spring only | Acceleration → Brake → Stop |
| **Best for** | Testing, safety mode | Simulators, active control |

---

### Q2: Do auto-centering parameters affect `auto_centering: false` mode?

**Answer**: **YES! This is important!**

When `auto_centering: false`, the system **still uses auto-centering parameters in the brake zone**:
- Once you enter the brake zone (within `brake_position` of target), it calls `calcCenteringForce()`
- This function uses `auto_centering_max_torque` and `auto_centering_max_position`
- So changing `auto_centering_max_torque` from 0.3 → 0.8 will make braking **much stronger**!

**What this means**:
- `auto_centering_max_torque` = Your **brake force strength** in manual control mode
- `auto_centering_max_position` = Your **brake force curve** in manual control mode

---

### Q2b: Do brake parameters affect `auto_centering: true` mode?

**Answer**: **NO**. When `auto_centering: true`:
- `brake_torque` and `brake_position` are **completely ignored**
- System never calls `calcRotateForce()` where these parameters are used
- Only auto-centering parameters matter

---

### Q3: Why does it feel similar near the target with both modes?

**Answer**: Because **both modes use the same centering force function** (`calcCenteringForce()`) near the target!

- **`auto_centering: true`**: Uses centering force everywhere
- **`auto_centering: false`**: Uses centering force only in brake zone

The difference is **when/where** it's applied, not the force calculation itself.

---

### Q4: Does auto-centering work with ROS messages?

**Answer**: **Yes**, but the behavior is different:

**`auto_centering: true`**:
- Wheel centers to `target.position` from ROS message
- If you publish `position: 0.3`, it centers to 135°, not 0°!
- If no messages, defaults to 0°

**`auto_centering: false`**:
- Wheel follows ROS commands with acceleration/brake/stop phases
- More precise control

---

### Q5: Which mode should I use?

**Answer**:

**Use `auto_centering: false` for**:
- Driving simulators ✅
- Real-time control ✅
- Dynamic target positions ✅
- Precise positioning ✅

**Use `auto_centering: true` for**:
- Hardware testing ✅
- Safety/fallback mode ✅
- Standalone operation ✅
- Simple demonstrations ✅

---

### Q6: How do I test auto-centering parameters?

**Answer**: See [Testing Guide](#testing-guide) above. Key steps:

1. Set `auto_centering: true`
2. Change `auto_centering_max_position` values (0.1, 0.2, 0.4)
3. Restart node after each change
4. Manually turn wheel and feel the difference

---

### Q7: Can I switch modes at runtime?

**Answer**: Currently requires node restart after changing config. For dynamic switching, you would need to implement ROS parameter updates or dynamic reconfigure.

---

## Parameter Summary Table

| Parameter | Used in `true` Mode | Used in `false` Mode | Description |
|-----------|---------------------|----------------------|-------------|
| **`auto_centering`** | Enables mode | Disables mode | Master switch |
| **`auto_centering_max_torque`** | ✅ Max force | ✅ Max brake force | Centering strength |
| **`auto_centering_max_position`** | ✅ Force curve | ✅ Brake force curve | Steepness control |
| **`min_torque`** | ✅ Baseline | ✅ Baseline | Minimum force |
| **`eps`** | ✅ Dead zone | ✅ Dead zone | Precision threshold |
| **`brake_torque`** | ❌ Ignored | ✅ Brake strength | Deceleration multiplier |
| **`brake_position`** | ❌ Ignored | ✅ Brake distance | When to start braking |
| **`max_torque`** | ✅ Safety limit | ✅ Safety limit | Final clamp (all modes) |

---

## Related Documentation

- [Configuration Guide](CONFIGURATION_GUIDE.md) - Complete parameter reference
- [Torque Behavior](TORQUE_BEHAVIOR.md) - `max_torque` and `min_torque` details
- [Braking Behavior](BRAKING_BEHAVIOR.md) - Brake system deep dive
- [Source Code](../src/g29_force_feedback.cpp) - Implementation details

---

## Summary

### Key Takeaways

1. **Auto-centering has two modes**: Always-on centering vs. brake-zone-only centering
2. **Same force function, different triggers**: Both modes use `calcCenteringForce()`
3. **Target position**: Centers to 0° by default or last ROS command
4. **Parameter overlap**: `auto_centering_max_torque` affects braking in BOTH modes!
5. **Force curve control**: `max_position` determines steepness, `max_torque` determines strength

### Quick Reference

**For simulator/control applications**:
```yaml
auto_centering: false
brake_torque: 0.8
brake_position: 0.1
eps: 0.02
```

**For testing/safety**:
```yaml
auto_centering: true
auto_centering_max_torque: 0.4
auto_centering_max_position: 0.2
```

Adjust from these baselines based on your specific needs!