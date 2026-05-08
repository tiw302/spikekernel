# WRO PROFESSIONAL API REFERENCE
A complete technical guide to all functions and classes in the framework.

---

## [0] SYSTEM SPECIFICATIONS
Before coding, understand the following standards used in this framework:

### Units
- **Distance**: Centimeters (cm)
- **Angle**: Degrees (deg) from -180 to 180
- **Speed**: Power units (0 to 1000)

### Coordinate System
- **X-Axis**: Forward / Backward movement.
- **Y-Axis**: Left / Right movement.
- **Heading (0 deg)**: Points towards the positive X-axis (Forward).
- **Rotation**: Positive (+) is Counter-Clockwise (Left), Negative (-) is Clockwise (Right).

### Best Practices
- **Stationary Gyro**: Do NOT touch or move the robot during the first 5 seconds of `full_setup`.
- **Battery**: Always run missions with >7.4V (BAT_WARN_MV) for consistent PID performance.
- **Pairing**: `motor_pair.pair()` is handled automatically by `setup.py`, do not re-pair manually unless necessary.

---
## [1] GETTING STARTED
To use this library in your mission script (e.g., `main.py`), follow this pattern:

1. **Import the library**: `import pid_lib as P` and `import setup`.
2. **Initialize**: Call `setup.full_setup()` to calibrate sensors and gyro.
3. **Loop**: Use the returned `odo` object for all movement commands.

**Example Code (Professional Pattern):**
```python
import setup, pid_lib as P

async def run_mission():
    # 1. Check if battery is healthy (Warning only)
    if not await setup.check_battery(warn_only=True):
        print("CRITICAL: Battery too low!")
        return

    # 2. Run initialization (Gyro + Sensors + Wall Align)
    # This returns None if hardware check fails
    odo = await setup.full_setup()
    
    if odo is not None:
        # 3. Start Mission
        await P.straight(odo, 50)
        await P.turn(odo, 90)
    else:
        print("SETUP FAILED: Check hardware or sensors.")
```

---

## [2] MOTION ENGINE (pid_lib)
Core movement and navigation routines.

### straight(odo, dist_cm, vmax=None, heading=None)
Move straight with S-Curve acceleration and PD steering.
- `dist_cm`: Distance in centimeters (negative for backward).
- `heading`: Maintain this absolute angle (default: current heading).

### turn(odo, target_h, vmax=None, max_ms=3000)
Rotate to an absolute heading using PD control.
- `target_h`: Target angle (-180 to 180).

### pivot_turn(odo, target_h, side='auto', vmax=None)
Turn using only one wheel (swings around the locked wheel).
- `side`: 'left', 'right', or 'auto'.

### swing_turn(odo, target_h, outer_speed=None, inner_ratio=0.0)
Smooth arc turn where both wheels move at different speeds.

### straight_with_motor(odo, dist_cm, vmax, arm_port, arm_target_deg)
Drive straight while moving an attachment motor simultaneously.
- Useful for picking up objects while moving to save time.

---

## [3] ATTACHMENT CONTROL
Routines for controlling robotic arms and claws.

### motor_run_degrees(p, target_deg, speed, tol=2)
Move a single motor to a specific relative position.
- `p`: Port (e.g., `C.PORT_C1`).
- `target_deg`: Relative target in degrees.

### motor_run_until_stall(p, speed, max_ms=3000, stall_ms=200)
Run motor until it hits resistance (useful for gripping or lowering arms).
- Returns `True` if stall was detected.

### motor_home(p, speed=150)
Run motor backward until it stalls and reset its position to 0.

---

## [4] PROFESSIONAL UTILITIES
Diagnostic and helper routines.

### wall_align(speed=200, max_ms=1500)
Drive backward into a wall to square the robot and reset the heading.

### comp_check()
Diagnostic: checks battery %, gyro noise, straight line accuracy, and 360-turn error.

### calibrate_gyro(preheat_ms=5000, samples=300)
Calculate gyro bias. Robot MUST be stationary.

---

## [5] PERCEPTION (sensor_lib)
Light/Color sensor processing.

### CAL.sweep(port)
Auto-calibration: Find Black/White values automatically.

### LineEst.dual()
Get normalized line error (-1.0 to 1.0) using two light sensors.
