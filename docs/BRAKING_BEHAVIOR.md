# G29 Braking System Behavior

## Table of Contents

- [Overview](#overview)
- [System Parameters](#system-parameters)
- [Three-Phase Control System](#three-phase-control-system)
- [Parameter Deep Dive](#parameter-deep-dive)
- [Configuration Examples](#configuration-examples)
- [Tuning Guide](#tuning-guide)
- [Distance Reference Table](#distance-reference-table)
- [Real-World Testing Results](#real-world-testing-results)

---

## Overview

The braking system controls how the G29 wheel approaches and stops at a target position. It uses a **three-phase control strategy** to prevent oscillation and overshooting:

1. **Acceleration Phase**: Drive toward target at full speed
2. **Braking Phase**: Apply reverse torque to decelerate
3. **Dead Zone Phase**: Zero torque to allow settling

This system is implemented in the [`calcRotateForce()`](../src/g29_force_feedback.cpp:134) function.

---

## System Parameters

### `brake_torque`
- **Type**: `double` (normalized)
- **Default**: `0.2`
- **Range**: `0.0` to `1.0`
- **Description**: Multiplier for braking force intensity
- **Formula**: `actual_brake_force = target.torque × brake_torque × -direction`

**What it controls**: How strong the braking force is when entering the brake zone.

### `brake_position`
- **Type**: `double` (normalized)
- **Default**: `0.1`
- **Range**: `0.0` to `1.0`
- **Description**: Distance threshold where braking begins
- **Formula**: `brake_zone_degrees = brake_position × 450°` (for G29)

**What it controls**: How far from the target the braking phase starts.

### `eps`
- **Type**: `double` (normalized)
- **Default**: `0.05`
- **Range**: `0.001` to `0.1`
- **Description**: Dead zone where wheel is considered "at target"
- **Formula**: `dead_zone_degrees = eps × 450°` (for G29)

**What it controls**: How close to the target the wheel must be before stopping all force.

### ⚠️ Critical Configuration Requirement

**IMPORTANT**: `eps` **MUST** be smaller than `brake_position` for the system to work correctly.

**Required relationship**: `eps < brake_position`

#### Why This Matters

The code checks conditions in priority order (see [`src/g29_force_feedback.cpp:142-154`](../src/g29_force_feedback.cpp:142-154)):

```cpp
if (fabs(diff) < m_eps) {
    // Dead zone - checked FIRST
    torque = 0.0;
    
} else if (fabs(diff) < m_brake_position) {
    // Brake zone - checked SECOND
    torque = target.torque * m_brake_torque * -direction;
    
} else {
    // Acceleration zone - default
    torque = target.torque * direction;
}
```

If `eps ≥ brake_position`, the dead zone check will **always trigger before** the brake zone check, completely disabling the braking phase!

#### What Happens with Invalid Configuration

**Invalid Example** (`eps ≥ brake_position`):
```yaml
brake_position: 0.1    # 45° brake zone
eps: 0.15              # 67.5° dead zone ❌ WRONG!
```

**Problem**:
- Distance 40° from target (0.089 normalized)
- Check 1: Is 0.089 < 0.15 (eps)? **YES** → Apply 0.0 torque (dead zone)
- Check 2: Never reached! (skipped because first condition was true)
- **Result**: Braking phase never activates, severe overshoot! ❌

#### Valid Configuration Examples

✅ **All of these work correctly**:
```yaml
# Example 1: Your optimal config
brake_position: 0.1
eps: 0.02         # 0.02 < 0.1 ✅

# Example 2: Tight control
brake_position: 0.08
eps: 0.01         # 0.01 < 0.08 ✅

# Example 3: Comfortable
brake_position: 0.15
eps: 0.05         # 0.05 < 0.15 ✅
```

❌ **These are BROKEN**:
```yaml
# Broken 1: eps equals brake_position
brake_position: 0.1
eps: 0.1          # 0.1 = 0.1 ❌ No brake zone!

# Broken 2: eps larger than brake_position
brake_position: 0.1
eps: 0.15         # 0.15 > 0.1 ❌ Brake zone disabled!
```

**Visual comparison**:

```
✅ CORRECT (eps < brake_position):
    Acceleration │ Brake │Dead│ Brake │ Acceleration
    ◄────────────┼───────┼────┼───────┼────────────►
              -0.1   -0.02  0  0.02  0.1
                ↑       ↑
         brake starts  dead zone

❌ WRONG (eps ≥ brake_position):
    Acceleration │ DEAD ZONE ONLY │ Acceleration
    ◄────────────┼────────────────┼────────────►
              -0.1               0.1
                ↑
         Brake zone swallowed by dead zone!
```

---

---

## Three-Phase Control System

### Phase 1: Acceleration (Far from Target)

**Condition**: 
```cpp
fabs(diff) >= m_brake_position
```

**Code** ([`src/g29_force_feedback.cpp:151-153`](../src/g29_force_feedback.cpp:151-153)):
```cpp
} else {
    torque = target.torque * direction;
    attack_length = m_loop_rate;
}
```

**Behavior**:
- **Distance from target**: Greater than `brake_position`
- **Torque applied**: `target.torque × direction` (full forward torque)
- **Direction**: Toward target (positive if target is right, negative if left)
- **Purpose**: Drive wheel to target at maximum commanded speed

**Example**:
- Target at 0° (center)
- Current position at -100° (100° left of center)
- `brake_position: 0.1` (45°)
- Distance: 100° > 45° → **Acceleration phase active**
- Published `torque: 0.8` → Applies **+0.8** (pushing right)

---

### Phase 2: Braking (Brake Zone)

**Condition**:
```cpp
m_eps < fabs(diff) < m_brake_position
```

**Code** ([`src/g29_force_feedback.cpp:146-149`](../src/g29_force_feedback.cpp:146-149)):
```cpp
} else if (fabs(diff) < m_brake_position) {
    m_is_brake_range = true;
    torque = target.torque * m_brake_torque * -direction;
    attack_length = m_loop_rate;
}
```

**Behavior**:
- **Distance from target**: Between `eps` and `brake_position`
- **Torque applied**: `target.torque × brake_torque × -direction` (reverse torque)
- **Direction**: **OPPOSITE** to movement (negative sign!)
- **Purpose**: Decelerate wheel to prevent overshoot
- **Note**: This sets `m_is_brake_range = true`, which switches to centering force mode in next loop

**Example**:
- Target at 0° (center)
- Current position at -30° (30° left of center)
- `brake_position: 0.1` (45°)
- `brake_torque: 1.0`
- `eps: 0.02` (9°)
- Distance: 30° is between 9° and 45° → **Braking phase active**
- Published `torque: 0.8` → Applies **-0.8** (pushing LEFT, opposing the rightward movement!)

---

### Phase 3: At Target (Dead Zone)

**Condition**:
```cpp
fabs(diff) < m_eps
```

**Code** ([`src/g29_force_feedback.cpp:142-144`](../src/g29_force_feedback.cpp:142-144)):
```cpp
if (fabs(diff) < m_eps) {
    torque = 0.0;
    attack_length = 0.0;
}
```

**Behavior**:
- **Distance from target**: Less than `eps`
- **Torque applied**: `0.0` (no force)
- **Purpose**: Allow wheel to settle without hunting/oscillation
- **Note**: Wheel can drift freely within this zone

**Example**:
- Target at 0° (center)
- Current position at +5° (5° right of center)
- `eps: 0.02` (9°)
- Distance: 5° < 9° → **Dead zone active**
- Torque applied: **0.0** (no force, wheel can freely move within ±9°)

---

## Parameter Deep Dive

### brake_torque - Braking Intensity

**Meaning**: Percentage of the requested torque used for braking force.

#### Effect of Changing Values

| Value | Braking Strength | Use Case | Pros | Cons |
|-------|------------------|----------|------|------|
| **0.0** | No braking | Not recommended | Fast movement | Severe overshoot, oscillation |
| **0.2** | 20% braking | Comfort-focused simulation | Smooth, gentle | May overshoot target |
| **0.5** | 50% braking | Balanced setup | Good compromise | Moderate precision |
| **0.8** | 80% braking | Precision control | Quick stops | Slightly abrupt |
| **1.0** | 100% braking | Maximum precision | No overshoot, fastest stop | Most abrupt feel |

#### Mathematical Example

**Scenario**: Wheel approaching target from left, published `torque: 0.6`

```
With brake_torque: 1.0
  Brake force = 0.6 × 1.0 × -1 = -0.6 (strong leftward push)

With brake_torque: 0.5
  Brake force = 0.6 × 0.5 × -1 = -0.3 (moderate leftward push)

With brake_torque: 0.2
  Brake force = 0.6 × 0.2 × -1 = -0.12 (weak leftward push)
```

The higher the `brake_torque`, the stronger the opposing force, resulting in faster deceleration.

---

### brake_position - Brake Zone Size

**Meaning**: How far from the target (in normalized coordinates) braking begins.

#### Effect of Changing Values

| Value | Brake Distance (degrees) | Effect | Use Case |
|-------|--------------------------|--------|----------|
| **0.05** | 22.5° | Very late braking | Fast response, risk of overshoot |
| **0.1** | 45° | Standard braking | Balanced (recommended) |
| **0.15** | 67.5° | Early braking | Gentle approach, slow |
| **0.2** | 90° | Very early braking | Ultra-smooth, very slow |

#### Visual Example

```
Target at 0° (center)

brake_position: 0.05 (22.5°)
├─────────────────┼─────────────────┤
-450°         -22.5°  0°  +22.5°    +450°
              └──brake──┘
              └─zone────┘

brake_position: 0.1 (45°)
├───────────┼─────────┼─────────┼───────────┤
-450°     -45°       0°       +45°       +450°
          └────brake zone─────┘

brake_position: 0.2 (90°)
├─────┼─────────────┼─────────────┼─────┤
-450° -90°         0°          +90°  +450°
      └──────brake zone──────────┘
```

---

### eps - Dead Zone (Precision Threshold)

**Meaning**: How close to the target is considered "close enough" to stop applying force.

#### Effect of Changing Values

| Value | Dead Zone (degrees) | Behavior | Result |
|-------|---------------------|----------|--------|
| **0.005** | ±2.25° | Ultra-tight | Very precise, likely oscillates |
| **0.01** | ±4.5° | Very tight | Precise, may oscillate |
| **0.02** | ±9° | **Optimal** | Good precision, no oscillation ✅ |
| **0.03** | ±13.5° | Moderate | Comfortable, some drift |
| **0.05** | ±22.5° | Loose | Smooth, significant drift |
| **0.1** | ±45° | Very loose | No oscillation, poor precision |

#### Testing Results

Based on real-world testing with G29 hardware:

```yaml
eps: 0.05  # Default - 22.5° drift
Result: ❌ Too much drift, wheel wanders within 45° range

eps: 0.02  # Tested optimal - 9° drift
Result: ✅ Sweet spot - precise positioning without oscillation

eps: 0.01  # Too tight - 4.5° drift
Result: ❌ Oscillates continuously, unstable
```

**Recommendation**: Use `eps: 0.02` for the best balance of precision and stability.

---

## Configuration Examples

### Example 1: Optimized Precision Setup (Tested & Recommended)

```yaml
brake_torque: 1.0      # Maximum braking force
brake_position: 0.1    # Standard brake distance
eps: 0.02              # Precise without oscillation
```

**Phase Breakdown**:
- **> 45° away**: Full acceleration toward target
- **9° to 45° away**: Full braking (100% reverse force)
- **< 9° away**: Zero force (settled)

**Behavior**:
```
Example: Moving from -100° to 0° (target) with torque: 0.8

Position -100° → -46°:  Apply +0.8 (accelerate right)
Position -45° → -10°:   Apply -0.8 (brake, push left)
Position -9° → +9°:     Apply 0.0 (settled, free movement)
```

**Use Case**: Robotics, research, precise control applications

**Characteristics**:
- ✅ Minimal overshoot
- ✅ Fast settling time
- ✅ Precise positioning (±9°)
- ⚠️ Abrupt stopping feel

---

### Example 2: Default Comfort Setup

```yaml
brake_torque: 0.2      # Gentle braking
brake_position: 0.1    # Standard brake distance
eps: 0.05              # Wide tolerance
```

**Phase Breakdown**:
- **> 45° away**: Full acceleration toward target
- **22.5° to 45° away**: Gentle braking (20% reverse force)
- **< 22.5° away**: Zero force (settled)

**Behavior**:
```
Example: Moving from -100° to 0° (target) with torque: 0.8

Position -100° → -46°:  Apply +0.8 (accelerate right)
Position -45° → -23°:   Apply -0.16 (gentle brake)
Position -22.5° → +22.5°: Apply 0.0 (wide settling zone)
```

**Use Case**: Casual driving simulation, comfortable gameplay

**Characteristics**:
- ✅ Smooth, comfortable feel
- ✅ No oscillation
- ❌ May overshoot target
- ❌ Wide drift range (±22.5°)

---

### Example 3: High-Precision Robotics Setup

```yaml
brake_torque: 0.8      # Strong braking
brake_position: 0.15   # Early braking
eps: 0.01              # Very tight tolerance
```

**Phase Breakdown**:
- **> 67.5° away**: Full acceleration toward target
- **4.5° to 67.5° away**: Strong braking (80% reverse force)
- **< 4.5° away**: Zero force (very precise)

**Behavior**:
```
Example: Moving from -100° to 0° (target) with torque: 0.8

Position -100° → -68°:  Apply +0.8 (accelerate right)
Position -67.5° → -5°:  Apply -0.64 (strong brake)
Position -4.5° → +4.5°: Apply 0.0 (tight settling zone)
```

**Use Case**: Research applications, high-accuracy positioning

**Characteristics**:
- ✅ Very precise (±4.5°)
- ✅ Long deceleration phase
- ⚠️ Risk of oscillation (test carefully!)
- ⚠️ Slower overall response

---

### Example 4: Racing Simulation Setup

```yaml
brake_torque: 0.4      # Moderate braking
brake_position: 0.08   # Late braking
eps: 0.03              # Moderate tolerance
```

**Phase Breakdown**:
- **> 36° away**: Full acceleration toward target
- **13.5° to 36° away**: Moderate braking (40% reverse force)
- **< 13.5° away**: Zero force (moderate settling)

**Behavior**:
```
Example: Moving from -100° to 0° (target) with torque: 0.8

Position -100° → -37°:  Apply +0.8 (long acceleration)
Position -36° → -14°:   Apply -0.32 (moderate brake)
Position -13.5° → +13.5°: Apply 0.0 (moderate drift range)
```

**Use Case**: Racing games, responsive steering feel

**Characteristics**:
- ✅ Quick response
- ✅ Natural racing feel
- ⚠️ Possible minor overshoot
- ⚠️ Moderate precision

---

## Tuning Guide

### Problem: Wheel Oscillates/Hunts Around Target

**Symptoms**: Wheel wobbles back and forth near target, never settles

**Solutions** (in priority order):
1. **Increase `eps`**: `0.01` → `0.02` → `0.03`
2. **Increase `brake_position`**: `0.1` → `0.15`
3. **Increase `brake_torque`**: `0.5` → `0.8` → `1.0`
4. **Decrease published torque**: Reduce force magnitude

**Example Fix**:
```yaml
# Before (oscillating)
eps: 0.01
brake_position: 0.1
brake_torque: 1.0

# After (stable)
eps: 0.02              # Wider dead zone prevents hunting
brake_position: 0.1    # Keep same
brake_torque: 1.0      # Keep same
```

---

### Problem: Wheel Overshoots Target

**Symptoms**: Wheel passes the target before coming back

**Solutions**:
1. **Increase `brake_torque`**: `0.2` → `0.5` → `1.0`
2. **Increase `brake_position`**: `0.1` → `0.15` (start braking earlier)
3. **Decrease `eps`**: `0.05` → `0.02` (tighter control)

**Example Fix**:
```yaml
# Before (overshooting)
brake_torque: 0.2      # Too weak
brake_position: 0.1
eps: 0.05

# After (precise stops)
brake_torque: 1.0      # Full braking strength
brake_position: 0.1    # Keep same
eps: 0.02              # Tighter control
```

---

### Problem: Wheel Drifts Too Much at Target

**Symptoms**: Wheel can move freely in a wide range around target

**Solutions**:
1. **Decrease `eps`**: `0.05` → `0.02` → `0.01`
2. **Monitor for oscillation** after reducing `eps`

**Example Fix**:
```yaml
# Before (drifting ±22.5°)
eps: 0.05

# After (precise ±9°)
eps: 0.02
```

---

### Problem: Stopping Feels Too Abrupt/Jerky

**Symptoms**: Wheel stops uncomfortably fast

**Solutions**:
1. **Decrease `brake_torque`**: `1.0` → `0.6` → `0.4`
2. **Increase `brake_position`**: `0.1` → `0.15` (gentler deceleration)

**Example Fix**:
```yaml
# Before (too abrupt)
brake_torque: 1.0
brake_position: 0.1

# After (smoother)
brake_torque: 0.4      # Gentler braking
brake_position: 0.15   # Longer deceleration phase
```

---

### Problem: Wheel Moves Too Slowly

**Symptoms**: Takes too long to reach target

**Solutions**:
1. **Decrease `brake_position`**: `0.15` → `0.1` → `0.08`
2. **Increase published torque** value
3. **Decrease `brake_torque`**: `1.0` → `0.7` (less braking resistance)

**Example Fix**:
```yaml
# Before (too slow)
brake_position: 0.15   # Braking starts too early

# After (faster)
brake_position: 0.08   # Brake later, move faster
```

---

## Distance Reference Table

### G29 Wheel - Normalized to Degrees Conversion

| Normalized Value | Degrees | Visual Reference | Common Use |
|------------------|---------|------------------|------------|
| **1.0** | 450° | Full right lock (1.25 rotations) | Maximum rotation |
| **0.5** | 225° | Half turn right | Sharp turn |
| **0.2** | 90° | Quarter turn right | Moderate turn |
| **0.15** | 67.5° | Large adjustment | Early brake zone |
| **0.1** | 45° | Eighth turn | Standard brake zone |
| **0.08** | 36° | Small turn | Late brake zone |
| **0.05** | 22.5° | Fine adjustment | Default dead zone |
| **0.03** | 13.5° | Very fine adjustment | Moderate dead zone |
| **0.02** | 9° | Tiny adjustment | **Optimal dead zone** ✅ |
| **0.01** | 4.5° | Minimal movement | Tight dead zone (oscillates) |
| **0.005** | 2.25° | Barely visible | Ultra-tight (unstable) |

### Practical Visualization

```
Full G29 Range: -450° to +450° (900° total rotation)

                    CENTER (0°)
                        |
    ◄───────────────────┼───────────────────►
   -450°              0°              +450°
   
Normalized coordinates: -1.0 to +1.0

   -1.0               0.0              +1.0
    ◄───────────────────┼───────────────────►
    
Example: brake_position = 0.1, eps = 0.02

    Acceleration │ Brake │Dead│ Brake │ Acceleration
    ◄────────────┼───────┼────┼───────┼────────────►
  -1.0        -0.1  -0.02  0  0.02  0.1          1.0
```

---

## Real-World Testing Results

### Test Setup
- **Hardware**: Logitech G29
- **Published torque**: `0.8`
- **Target position**: `0.0` (center)
- **Starting position**: `-0.2` (-90°)

### Test 1: Default Configuration

```yaml
brake_torque: 0.2
brake_position: 0.1
eps: 0.05
```

**Result**:
- Acceleration phase: -90° → -45° (smooth)
- Braking phase: -45° → -22.5° (very gentle, barely noticeable)
- Dead zone: -22.5° → +22.5° (wheel drifts freely, can wobble up to 45° total)
- **Verdict**: ❌ Too much drift, imprecise

---

### Test 2: Optimized Configuration (Recommended)

```yaml
brake_torque: 1.0
brake_position: 0.1
eps: 0.02
```

**Result**:
- Acceleration phase: -90° → -45° (smooth, full speed)
- Braking phase: -45° → -9° (strong, noticeable deceleration)
- Dead zone: -9° → +9° (minimal drift, stays very close to target)
- **Verdict**: ✅ Excellent - precise without oscillation

---

### Test 3: Too Tight (Oscillation Test)

```yaml
brake_torque: 1.0
brake_position: 0.1
eps: 0.01
```

**Result**:
- Acceleration phase: -90° → -45° (smooth)
- Braking phase: -45° → -4.5° (strong braking)
- Dead zone: -4.5° → +4.5° (wheel continuously hunts, oscillates ±5-10°)
- **Verdict**: ❌ Oscillates - dead zone too tight for hardware precision

---

### Test 4: Comfort Setup

```yaml
brake_torque: 0.4
brake_position: 0.15
eps: 0.03
```

**Result**:
- Acceleration phase: -90° → -67.5° (shorter acceleration)
- Braking phase: -67.5° → -13.5° (long, gentle deceleration)
- Dead zone: -13.5° → +13.5° (comfortable settling)
- **Verdict**: ✅ Smooth and comfortable, acceptable precision

---

## Summary

### Quick Reference

| Application | `brake_torque` | `brake_position` | `eps` |
|-------------|----------------|------------------|-------|
| **Precision Robotics** | 0.8 - 1.0 | 0.1 - 0.15 | 0.01 - 0.02 |
| **Research/Testing** | 1.0 | 0.1 | 0.02 |
| **Racing Simulation** | 0.4 - 0.6 | 0.08 - 0.1 | 0.02 - 0.03 |
| **Comfort Driving** | 0.3 - 0.5 | 0.1 - 0.15 | 0.03 - 0.05 |
| **Default (Safe)** | 0.2 | 0.1 | 0.05 |

### Recommended Starting Point (Tested Optimal)

```yaml
brake_torque: 1.0      # Maximum precision
brake_position: 0.1    # Balanced brake distance
eps: 0.02              # Optimal dead zone (no oscillation)
```

This configuration provides:
- ✅ Precise positioning (±9°)
- ✅ No oscillation
- ✅ Quick settling
- ✅ Minimal overshoot

Adjust from this baseline based on your specific application needs.

---

## Related Documentation

- [Configuration Guide](CONFIGURATION_GUIDE.md) - Complete parameter reference
- [Torque Behavior](TORQUE_BEHAVIOR.md) - `max_torque` and `min_torque` details
- [Source Code](../src/g29_force_feedback.cpp) - Implementation details