# Rivian Lane Centering Improvements

## Problem
The vehicle was drifting too much before openpilot corrected, allowing it to cross lane lines before intervention.

## Solution
Improved the torque-based lateral control tuning parameters to make the system more responsive and reduce drift.

## Changes Made

### File: `opendbc_repo/opendbc/car/rivian/interface.py`

**Modified Parameters:**

1. **Proportional Gain (kp)**: `1.0` → `1.5` (+50%)
   - **Effect**: Faster response to lane position errors
   - **Why**: Higher kp means the system reacts more quickly when it detects deviation from center

2. **Integral Gain (ki)**: `0.3` → `0.4` (+33%)
   - **Effect**: Better steady-state tracking and elimination of persistent errors
   - **Why**: Helps the system maintain center position over time, reducing drift

3. **Steering Angle Deadzone**: `0.0` → `0.1` degrees
   - **Effect**: Prevents overcorrection for very small errors
   - **Why**: Small deadzone allows the system to ignore tiny deviations while still responding to meaningful drift

**Note**: **Friction is NOT changed** - it remains at `0.07` (from `torque_data/override.toml`) because it is dynamically managed by live torque learning and should not be manually tuned.

## How These Parameters Work

### PID Controller
The lateral control uses a PID (Proportional-Integral-Derivative) controller:

- **Proportional (kp)**: Responds to current error - how far off center you are right now
- **Integral (ki)**: Responds to accumulated error - persistent drift over time
- **Feedforward (kf)**: Provides predictive torque based on desired lateral acceleration (for curves)
- **Friction**: Dynamically managed by live torque learning - provides resistance to lateral movement

### Expected Results

1. **Faster Correction**: The vehicle should respond more quickly when it starts to drift
2. **Better Centering**: The vehicle should maintain lane center more accurately
3. **Reduced Drift**: Less tendency to drift toward lane edges before correction
4. **Smoother Control**: Small deadzone prevents jittery corrections for tiny errors

## Testing Recommendations

1. **Highway Driving**: Test on straight highways to verify improved centering
2. **Curved Roads**: Ensure the system still handles curves smoothly
3. **Wind Conditions**: Test in crosswinds to verify drift reduction
4. **Different Speeds**: Verify behavior at various speeds (30-80 mph)

## Fine-Tuning (if needed)

If the system feels too aggressive or not responsive enough, you can adjust:

- **Too aggressive/jerky**: Reduce `kp` (try 1.2-1.3) or increase deadzone (try 0.15)
- **Still drifting**: Increase `ki` (try 0.45-0.5) or decrease `latAccelFactor` (try 2.6)
- **Too slow to respond**: Increase `kp` further (try 1.6-1.8)
- **Note**: Friction is managed by live torque learning and should not be manually adjusted

## Notes

- These changes only affect lateral (steering) control, not longitudinal (speed) control
- The changes are conservative and should work well for most driving conditions
- If you experience any issues, the original values can be restored

