# All Adjustable Parameters for Lane Centering

## Currently Changed Parameters ✅

1. **kp** (Proportional Gain): `1.0` → `1.5`
2. **ki** (Integral Gain): `0.3` → `0.4`
3. **steeringAngleDeadzoneDeg**: `0.0` → `0.1`

**Note**: `friction` is **NOT** changed - it remains at `0.07` (from `torque_data/override.toml`) because it is dynamically managed by live torque learning and should not be manually tuned.

## Additional Torque Tuning Parameters Available

### 1. **kf** (Feedforward Gain)
- **Current**: `1.0` (default)
- **Location**: `opendbc_repo/opendbc/car/rivian/interface.py`
- **What it does**: Provides **predictive torque** based on desired lateral acceleration. This is the "feedforward" component of the PID controller - it anticipates the torque needed for curves before the error occurs.
- **How it works**: When you approach a curve, the system calculates the desired lateral acceleration. The feedforward (kf) multiplies this desired acceleration to provide predictive steering torque. The formula is: `feedforward_torque = desired_lateral_accel * kf`
- **Effect**:
  - **Higher values (1.1-1.2)**: More aggressive feedforward, smoother on curves, better curve handling
  - **Lower values (0.8-0.9)**: Less aggressive feedforward, more reactive (waits for error)
- **When to adjust**: If the vehicle feels sluggish entering curves or overshoots curves
- **Suggested range**: `0.8` - `1.2`
- **Example**: `ret.lateralTuning.torque.kf = 1.1`

### 2. **latAccelFactor** (Lateral Acceleration Factor)
- **Current**: `2.8` (from `torque_data/override.toml` - Rivian-specific)
- **Location**: `opendbc_repo/opendbc/car/torque_data/override.toml`
- **What it does**: **Converts between lateral acceleration and steering torque**. This is the "calibration factor" that tells the system how much torque is needed to achieve a given lateral acceleration.
- **How it works**: The system works in lateral acceleration space (m/s²), but needs to convert to steering torque. The formula is: `torque = lateral_acceleration / latAccelFactor`. A higher factor means more torque is needed for the same acceleration.
- **Effect**:
  - **Lower values (2.5-2.6)**: More sensitive - less torque needed for same acceleration, system responds more aggressively
  - **Higher values (3.0-3.2)**: Less sensitive - more torque needed for same acceleration, system responds more gently
- **When to adjust**: If the system feels too sensitive (jerky) or not sensitive enough (sluggish)
- **Suggested range**: `2.5` - `3.2`
- **Note**: This is a **major tuning parameter** - small changes have significant effects

### 3. **latAccelOffset** (Lateral Acceleration Offset)
- **Current**: `0.0` (default)
- **Location**: `opendbc_repo/opendbc/car/rivian/interface.py`
- **What it does**: **Compensates for device roll misalignment**. If your Comma device is not perfectly level (tilted left or right), this can cause the vehicle to drift in one direction.
- **How it works**: The system measures lateral acceleration. If the device is tilted, it will read a false lateral acceleration even when going straight. This offset subtracts that bias: `adjusted_accel = measured_accel - latAccelOffset`
- **Effect**:
  - **Positive values (0.05-0.1)**: Compensates for device tilted right - vehicle will bias left to compensate
  - **Negative values (-0.05 to -0.1)**: Compensates for device tilted left - vehicle will bias right to compensate
  - **Zero (0.0)**: No compensation needed (device is level)
- **When to adjust**: If the vehicle consistently drifts to one side even on straight roads (and it's not a road crown issue)
- **Suggested range**: `-0.1` to `0.1`
- **Example**: If vehicle drifts right, try `latAccelOffset = 0.05` to compensate

## System-Level Parameters

### 4. **steerActuatorDelay**
- **Current**: `0.15` seconds
- **Location**: `opendbc_repo/opendbc/car/rivian/interface.py`
- **What it does**: Accounts for steering system delay
- **Effect**: Lower = more responsive, but can cause overshoot if too low
- **Suggested range**: `0.10` - `0.20`
- **Note**: Should match actual vehicle delay

### 5. **steerLimitTimer**
- **Current**: `0.4` seconds
- **Location**: `opendbc_repo/opendbc/car/rivian/interface.py`
- **What it does**: Time before steering limit alert
- **Effect**: Doesn't directly affect centering, but affects safety alerts

## Torque Data Parameters (in override.toml)

### 6. **MAX_LAT_ACCEL_MEASURED**
- **Current**: `2.5` m/s²
- **Location**: `opendbc_repo/opendbc/car/torque_data/override.toml`
- **What it does**: Maximum lateral acceleration the vehicle can achieve
- **Effect**: Higher = allows sharper turns, but may exceed vehicle limits
- **Suggested range**: `2.5` - `3.5`
- **Format**: `"RIVIAN_R1_GEN1" = [2.8, 3.0, 0.12]` (latAccelFactor, MAX_LAT_ACCEL, FRICTION)

## Extension Parameters (Advanced - in latcontrol_torque_ext_base.py)

These are in the extension class and can be modified if you create a custom extension:

### 7. **friction_look_ahead_v** (Lookahead Time)
- **Current**: `[1.4, 2.0]` seconds
- **Location**: `sunnypilot/selfdrive/controls/lib/latcontrol_torque_ext_base.py`
- **What it does**: How far ahead to look for lateral jerk
- **Effect**: Higher = more predictive, smoother
- **Suggested range**: `[1.2, 1.8]` to `[1.6, 2.2]`

### 8. **friction_look_ahead_bp** (Lookahead Speed Breakpoints)
- **Current**: `[9.0, 30.0]` m/s
- **Location**: `sunnypilot/selfdrive/controls/lib/latcontrol_torque_ext_base.py`
- **What it does**: Speed breakpoints for lookahead interpolation
- **Effect**: Adjusts when lookahead changes with speed

### 9. **lat_jerk_friction_factor**
- **Current**: `0.4`
- **Location**: `sunnypilot/selfdrive/controls/lib/latcontrol_torque_ext_base.py`
- **What it does**: Scales lateral jerk contribution to friction
- **Effect**: Higher = more responsive to jerk changes
- **Suggested range**: `0.3` - `0.6`

### 10. **lat_accel_friction_factor**
- **Current**: `0.7`
- **Location**: `sunnypilot/selfdrive/controls/lib/latcontrol_torque_ext_base.py`
- **What it does**: Scales lateral acceleration error contribution to friction
- **Effect**: Higher = stronger friction response
- **Suggested range**: `0.5` - `1.0`

## Global Constants (Less Common to Change)

### 11. **FRICTION_THRESHOLD**
- **Current**: `0.3` m/s²
- **Location**: `opendbc_repo/opendbc/car/lateral.py`
- **What it does**: Threshold for friction compensation activation
- **Effect**: Lower = friction activates sooner
- **Note**: Changing this affects all cars, not recommended

## Recommended Next Steps for Fine-Tuning

### If Still Drifting Too Much:
1. Increase `ki`: `0.4` → `0.45` or `0.5`
2. Decrease `latAccelFactor`: `2.8` → `2.6` (makes system more sensitive)
3. Decrease `steeringAngleDeadzoneDeg`: `0.1` → `0.05` (responds to smaller errors)
4. Increase `kf`: `1.0` → `1.1` (better feedforward for curves)

### If Too Aggressive/Jerky:
1. Decrease `kp`: `1.5` → `1.2` or `1.3`
2. Increase `steeringAngleDeadzoneDeg`: `0.1` → `0.15` or `0.2`
3. Increase `latAccelFactor`: `2.8` → `3.0` (makes system less sensitive)
4. Decrease `kf`: `1.0` → `0.9` (less aggressive feedforward)

### If Slow to Respond:
1. Increase `kp`: `1.5` → `1.7` or `1.8`
2. Decrease `steeringAngleDeadzoneDeg`: `0.1` → `0.05` or `0.0`
3. Decrease `latAccelFactor`: `2.8` → `2.6` (more sensitive)
4. Adjust `kf`: `1.0` → `1.1` or `1.2` (better feedforward)

### For Better Curve Handling:
1. Increase `kf`: `1.0` → `1.1` or `1.2`
2. Adjust `latAccelFactor`: Try `2.6` - `3.0` range
3. Increase `MAX_LAT_ACCEL_MEASURED`: `2.5` → `3.0` (if vehicle can handle it)

## Example: Comprehensive Tuning

```python
# In opendbc_repo/opendbc/car/rivian/interface.py

CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.1)

# Comprehensive tuning
ret.lateralTuning.torque.kp = 1.5
ret.lateralTuning.torque.ki = 0.4
ret.lateralTuning.torque.kf = 1.1  # Better feedforward
# Note: friction is managed by live torque learning, don't override
ret.lateralTuning.torque.latAccelFactor = 2.7  # Override from torque_data
ret.lateralTuning.torque.latAccelOffset = 0.0  # Adjust if needed for roll compensation
```

## Parameter Priority (Most Impact → Least Impact)

1. **kp** - Directly affects response speed ⭐⭐⭐
2. **ki** - Improves steady-state tracking ⭐⭐
3. **latAccelFactor** - Affects sensitivity ⭐⭐
4. **steeringAngleDeadzoneDeg** - Prevents overcorrection ⭐
5. **kf** - Improves curve handling ⭐
6. **latAccelOffset** - Fine-tuning for roll compensation ⭐
7. **friction** - Managed by live torque learning (not tunable) ⚠️

## Notes

- Start with the 3 parameters we already changed (kp, ki, deadzone)
- **Friction is NOT tunable** - it's dynamically managed by live torque learning
- Test thoroughly before adjusting more
- Change one parameter at a time to understand its effect
- Some parameters (like `steerActuatorDelay`) should match your vehicle's actual characteristics
- Extension parameters require creating a custom extension class

