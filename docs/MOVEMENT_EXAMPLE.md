# G29 Force Feedback Movement Example

## Overview

This document explains the complete behavior of the G29 wheel through a detailed example, showing how forces are applied during movement from one position to another.

**Key concept**: In the braking phase, the force pushes **opposite** to movement direction to slow down the wheel - just like car brakes!

---

## Example Configuration

```yaml
auto_centering: false
max_torque: 1.0
min_torque: 0.2
brake_torque: 0.2
brake_position: 0.1    # 45° from target
eps: 0.02              # 9° dead zone
auto_centering_max_torque: 0.3
auto_centering_max_position: 0.2  # 90° from target
```

---

## Scenario: Moving from 135° → 0° (Center)

**Published command**:
```bash
ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
  "{position: 0.0, torque: 0.8}"
```

**Setup**:
- **Current position**: 135° (right of center)
- **Target position**: 0° (center)
- **Required movement**: LEFTWARD (from right to center)
- **Published torque**: 0.8

---

## The Four Phases of Movement

### Phase 1: Acceleration Phase (135° → 45° from target)

**Wheel position**: 135° → 90° → 45°

**Distance from target**: 135° → 90° → 45° (all > 45°)

**Code used**: [`calcRotateForce()`](../src/g29_force_feedback.cpp:151-153)
```cpp
torque = target.torque * direction;  // 0.8 × -1 = -0.8
```

**Force applied**: **-0.8 (LEFTWARD push)**

**Force direction**: ⬅️ SAME as movement direction

**What happens**:
```
Position:  135° ────────→ 90° ────────→ 45°
                ⬅️         ⬅️         ⬅️
Force:      PUSH LEFT   PUSH LEFT   PUSH LEFT
Magnitude:     -0.8        -0.8        -0.8

Result: Wheel accelerates LEFTWARD toward target
```

**Physical feeling**: 
- Strong force helping you turn left
- Wheel moves quickly
- Momentum builds up

**Analogy**: Like pressing the gas pedal - car accelerates in the direction you want to go

---

### Phase 2: Brake Trigger (At 45° from target)

**Wheel position**: 45° (right of center)

**Distance from target**: Exactly 45°

**Wheel momentum**: Still moving LEFTWARD

**Code used**: [`calcRotateForce()`](../src/g29_force_feedback.cpp:146-149)
```cpp
m_is_brake_range = true;
torque = target.torque * m_brake_torque * -direction;
// = 0.8 × 0.2 × -(-1) = +0.16
```

**Force applied**: **+0.16 (RIGHTWARD push)**

**Force direction**: ➡️ OPPOSITE to movement direction

**What happens**:
```
Position:      45° ───────→ 44° ───────→ 43°
Movement:        ⬅️ LEFT     ⬅️ LEFT     ⬅️ LEFT
Force:          PUSH RIGHT → → →
Magnitude:         +0.16

Result: Wheel STILL moves LEFT, but starts slowing down
        (Force opposes movement = BRAKING!)
```

**Physical feeling**:
- Suddenly feel resistance
- Wheel still turns left, but fighting against force
- Speed starts decreasing

**Analogy**: Like tapping the brake pedal - car still moves forward, but starts slowing down

---

### Phase 3: Progressive Centering Brake (45° → 9° from target)

**Wheel position**: 45° → 25° → 10°

**Distance from target**: 45° → 25° → 10° (between 45° and 9°)

**Wheel momentum**: Moving LEFTWARD but slowing down

**Code used**: [`calcCenteringForce()`](../src/g29_force_feedback.cpp:168-172) (triggered by `m_is_brake_range = true`)

**Force calculations**:

**At 45° from target**:
```
power = (0.1 - 0.02) / (0.2 - 0.02) = 0.444
torque = 0.444 × (0.3 - 0.2) + 0.2 = 0.244
Applied: +0.244 RIGHTWARD
```

**At 25° from target**:
```
power = (0.055 - 0.02) / (0.2 - 0.02) = 0.194
torque = 0.194 × (0.3 - 0.2) + 0.2 = 0.219
Applied: +0.219 RIGHTWARD
```

**At 10° from target**:
```
power = (0.022 - 0.02) / (0.2 - 0.02) = 0.011
torque = 0.011 × (0.3 - 0.2) + 0.2 = 0.201
Applied: +0.201 RIGHTWARD
```

**Force direction**: ➡️ OPPOSITE to movement direction (progressively weaker)

**What happens**:
```
Position:  45° ────────→ 25° ────────→ 10°
             ⬅️ LEFT      ⬅️ LEFT      ⬅️ LEFT
Movement: (slowing)   (slower)    (very slow)
             
Force:    PUSH RIGHT  PUSH RIGHT  PUSH RIGHT
           → → →      → → →      → → →
Magnitude:  +0.244      +0.219      +0.201

Result: Wheel continues LEFT but decelerates smoothly
        (Progressive braking prevents overshoot)
```

**Physical feeling**:
- Moderate resistance against leftward movement
- Resistance gets gradually weaker as you approach center
- Smooth, controlled deceleration
- Like gentle, progressive braking

**Analogy**: Like gradually releasing brake pressure as you approach a stop sign - smooth deceleration

---

### Phase 4: Dead Zone (< 9° from target)

**Wheel position**: 8° → 5° → 2° → 0°

**Distance from target**: < 9° (very close)

**Code used**: [`calcRotateForce()`](../src/g29_force_feedback.cpp:142-144)
```cpp
if (fabs(diff) < m_eps) {
    torque = 0.0;
}
```

**Force applied**: **0.0 (NO FORCE)**

**What happens**:
```
Position:    8° ───→ 5° ───→ 2° ───→ 0°
              ⬅️       ⬅️       ⬅️      ⬛
Movement:  (coasting)(coast)(coast)(stop)

Force:      NONE    NONE    NONE    NONE
Magnitude:   0.0     0.0     0.0     0.0

Result: Wheel settles at target, can drift ±9°
```

**Physical feeling**:
- No resistance
- Wheel feels "free" within ±9° range
- Natural settling

**Analogy**: Like parking - you've stopped, engine off, wheel can move slightly

---

## Complete Journey Visualization

### Timeline View

```
TIME →
═══════════════════════════════════════════════════════════════════════════

PHASE 1: ACCELERATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Position:    135° ────────────────────────→ 45°
Movement:      ⬅️⬅️⬅️⬅️⬅️⬅️⬅️⬅️⬅️⬅️⬅️⬅️⬅️⬅️⬅️⬅️⬅️⬅️
Force:        PUSH LEFT (-0.8)
Feel:         STRONG ACCELERATION

PHASE 2: BRAKE TRIGGER (one moment)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Position:    45° → 44.5°
Movement:      ⬅️ (still leftward, but slowing)
Force:        PUSH RIGHT (+0.16) ➡️
Feel:         RESISTANCE STARTS

PHASE 3: PROGRESSIVE BRAKE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Position:    45° ───────────────────────→ 9°
Movement:      ⬅️⬅️⬅️⬅️ (slowing down) ⬅️⬅️⬅️
Force:        PUSH RIGHT (+0.24 → +0.20) ➡️➡️➡️
Feel:         SMOOTH DECELERATION

PHASE 4: DEAD ZONE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Position:    9° ───→ 0° (settled)
Movement:      ⬅️ ⬅️  🛑
Force:        NONE (0.0)
Feel:         FREE MOVEMENT, SETTLED

═══════════════════════════════════════════════════════════════════════════
```

---

## Force vs Position Graph

```
Applied Force (direction)
    
  -0.8 |█████████████████████████┐         
 LEFT  |                         │         
       |                         │  Acceleration
  -0.4 |                         │  (push LEFT)
       |                         │         
   0.0 |─────────────────────────┴─┬───────┬─────
       |                           │  Brake│Dead 
       |                          ╱│       │zone
 RIGHT |                         ╱ │       │
       |                        ╱  │       │
  +0.2 |                    ┌──┘   │       │
       |                  ╱         │       │
  +0.4 |                ╱           │       │
       |              ╱  Brake      │       │
       |____________╱________________│_______│_____
           135°   45°              9°      0°
                  Position (degrees from target)

Legend:
  Negative (left) = Push helping movement
  Positive (right) = Push opposing movement (BRAKING)
```

---

## Understanding Brake Force Direction

### Why Does Brake Force Push OPPOSITE to Movement?

**Think of a car**:

```
Car moving forward at 60 mph:
┌─────────────────────────────────────┐
│        🚗 ───→                      │  Car moves FORWARD
│ Wheels rotating forward             │
└─────────────────────────────────────┘

Press brake pedal:
┌─────────────────────────────────────┐
│        🚗 ──→                       │  Car STILL moves forward
│ Brake pads push ←── BACKWARD        │  (but slowing down)
│ (opposes wheel rotation)            │
└─────────────────────────────────────┘

Result: Car decelerates but continues forward
```

**Same with steering wheel**:

```
Wheel moving LEFT at high speed:
┌─────────────────────────────────────┐
│   ⬅️⬅️⬅️⬅️ Wheel rotating LEFT       │
│                                     │
└─────────────────────────────────────┘

Brake zone activated:
┌─────────────────────────────────────┐
│   ⬅️⬅️⬅️ Wheel STILL rotating LEFT   │
│   Motor pushes ➡️➡️➡️ RIGHT          │
│   (opposes rotation)                │
└─────────────────────────────────────┘

Result: Wheel decelerates but continues LEFT
```

---

## Key Insights

### 1. **Force Direction ≠ Movement Direction (During Braking)**

| Phase | Movement Direction | Force Direction | Purpose |
|-------|-------------------|-----------------|---------|
| **Acceleration** | ⬅️ LEFT | ⬅️ LEFT (same) | Speed up |
| **Braking** | ⬅️ LEFT | ➡️ RIGHT (opposite) | Slow down |
| **Dead zone** | 🛑 STOPPED | No force | Settled |

---

### 2. **The Brake Force is NOT Trying to Move You Somewhere**

❌ **WRONG thinking**: "Rightward force means wheel goes right"

✅ **CORRECT thinking**: "Rightward force opposes my leftward movement to slow me down"

---

### 3. **Published Torque vs Auto-Centering Max Torque**

**Your published torque (0.8)** is only used in acceleration:
- Phase 1: Uses 0.8 ✅

**Auto-centering max torque (0.3)** controls braking:
- Phase 3: Uses 0.3 (ignores your 0.8!) ⚠️

**Result**: Acceleration is stronger than braking → possible overshoot

---

## Common Misunderstandings

### ❌ Misunderstanding 1: "Why does it push right when I want to go left?"

**Answer**: It's **braking**, not **driving**!

- Phase 1-2: You've been accelerating LEFT
- Your wheel has LEFTWARD momentum
- Phase 3: To SLOW DOWN this leftward movement, force must push OPPOSITE (RIGHT)
- Wheel **still moves LEFT**, just slower

---

### ❌ Misunderstanding 2: "The force direction changes randomly"

**Answer**: Force direction changes based on **function**:

- **Acceleration**: Force pushes TOWARD target (helps movement)
- **Braking**: Force pushes AWAY from target (opposes movement)

This is by design to create smooth, controlled motion!

---

### ❌ Misunderstanding 3: "Auto-centering parameters don't affect manual mode"

**Answer**: They DO affect braking!

When `auto_centering: false`:
- `auto_centering_max_torque` = **Your brake strength**
- `auto_centering_max_position` = **Your brake force curve**

Even though auto-centering is disabled, these parameters control braking behavior!

---

## Real-World Analogy Summary

```
┌──────────────────────┬─────────────────┬──────────────────────┐
│ Phase                │ Steering Wheel  │ Car Analogy          │
├──────────────────────┼─────────────────┼──────────────────────┤
│ 1. Acceleration      │ Motor pushes    │ Press gas pedal      │
│                      │ toward target   │ (accelerate forward) │
├──────────────────────┼─────────────────┼──────────────────────┤
│ 2. Brake trigger     │ Detect close    │ See stop sign ahead  │
│                      │ to target       │                      │
├──────────────────────┼─────────────────┼──────────────────────┤
│ 3. Progressive brake │ Motor opposes   │ Gradually press      │
│                      │ movement        │ brake pedal          │
│                      │ (push opposite) │ (car slows, still    │
│                      │                 │  moves forward)      │
├──────────────────────┼─────────────────┼──────────────────────┤
│ 4. Dead zone         │ No force,       │ Stopped, engine off  │
│                      │ settled         │ (can roll slightly)  │
└──────────────────────┴─────────────────┴──────────────────────┘
```

---

## Testing This Yourself

### Test 1: Feel the Force Direction Change

**Config**:
```yaml
auto_centering: false
brake_position: 0.1
auto_centering_max_torque: 0.5
eps: 0.02
```

**Command**:
```bash
ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback \
  "{position: 0.0, torque: 0.8}"
```

**What to observe**:
1. Start at 135° (right of center)
2. **Feel strong LEFT pull** (acceleration)
3. **At ~45°, feel RIGHT resistance** (braking starts)
4. **Wheel still turns LEFT** but fights the force
5. **Gentle stop near 0°**

---

### Test 2: Compare Different Brake Strengths

**Weak braking**:
```yaml
auto_centering_max_torque: 0.2  # Weak brake
```
Result: May overshoot, wheel passes 0° before coming back

**Strong braking**:
```yaml
auto_centering_max_torque: 0.8  # Strong brake
```
Result: Precise stop, minimal overshoot

---

## Summary

**The complete journey**:
1. **Acceleration**: Force helps you move toward target (135° → 45°)
2. **Braking**: Force opposes movement to slow you down (45° → 9°)
3. **Settling**: No force, wheel at target (< 9°)

**Key concept**: During braking, force pushes **opposite** to movement - this is how brakes work! The wheel still moves toward the target, just **slower**.

Think of it like driving a car - when you brake, the car doesn't go backward, it just slows down while still moving forward. Same principle applies here! 🚗🛑