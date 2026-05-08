# WRO PROFESSIONAL API REFERENCE
```yaml
      ___________________
     |      SPIKE        |
     |   [  O  ]  HUB    |
     |___________________|
     |  A  B  C  D  E  F |
     '-------------------'
```
A complete technical guide to all functions and classes in the framework.

**English Edition** | [ภาษาไทย](API_REFERENCE_TH.md)

---

## [0] MISSION CHECKLIST (PRE-LAUNCH)
Follow these 4 steps before every official run to ensure maximum reliability.

1. **Battery Check**: Ensure voltage is > 7.8V for consistent PID motor performance.
2. **Gyro Calibration**: Run `setup.full_setup()`. Keep the robot **perfectly still** for 5 seconds.
3. **Sensor Calibration**: Use `CAL.sweep()` or `setup.sensor_setup()` on the actual competition mat.
4. **Origin Alignment**: Use `wall_align()` or manual placement to ensure X, Y, and Heading are perfectly set to 0.

---

## [1] GETTING STARTED

**Example Code (Professional Pattern):**
```python
import setup, pid_lib as P

async def run_mission():
    # 1. Check if battery is healthy (Warning only)
    if not await setup.check_battery(warn_only=True):
        print("CRITICAL: Battery too low!")
        return

    # 2. Run initialization (Gyro + Sensors + Wall Align)
    odo = await setup.full_setup()
    
    if odo is not None:
        # 3. Start Mission
        await P.straight(odo, 50)
        await P.turn(odo, 90)
```

---

## [2] BASIC MOTION (pid_lib)

### straight(odo, dist_cm)
Drive Forward (+) or Backward (-) in a straight line.
- **`odo`**: The Odometry object tracking the robot's position.
- **`dist_cm`**: Distance to move in centimeters (Positive = Forward, Negative = Backward).

```yaml
Visual: |
      [  FINISH  ]
      +----------+
      | [----]   |
      | |  O |   |
      | [----]   |
      +----------+
           /|\
            | (Move Straight)
            |
      +----------+
      | [----]   |
      | |  O |   |
      | [----]   |
      +----------+
      [  START   ]
```

### turn(odo, target_h)
Rotate the robot on its center axis to a specific global angle.
- **`odo`**: The Odometry object.
- **`target_h`**: Absolute target heading in degrees (-180 to 180).

```yaml
Visual: |
      +----------+
      |  [ || ]  |
      |  [  O ]  | (Rotate 90)
      |  [ || ]  |
      +----------+
           /|\
            | (Turn on Spot)
            |
      +----------+
      |  [----]  |
      |  |  O |  | (Start 0)
      |  [----]  |
      +----------+
```

### pivot_turn(odo, target_h, side)
Turn by locking one wheel and swinging the other.
- **`odo`**: The Odometry object.
- **`target_h`**: Absolute target heading in degrees.
- **`side`**: The wheel to keep stationary. Options: `'left'`, `'right'`, or `'auto'` (shortest path).

```yaml
Visual: |
               +------+
               | [||] |
               | [ O] |
               | [||] |
               +------+
              /
             / (Swing Path)
            /
      +----------+
      |  [FIXED] |
      |  |[X] |  |  <-- Stationary wheel
      |  [----]  |
      +----------+
```

---

## [3] ADVANCED SENSOR MOTION (sensor_lib)

### lf_n_junctions(n, vmax)
Follow line and stop exactly at the Nth junction (Intersection).
- **`n`**: Total number of junctions to cross before stopping.
- **`vmax`**: Maximum speed for the line following (0-1000).

```yaml
Visual: |
      [ STOP AT #N ]
      +----------+
      |  [----]  |
      |  |  O |  |
      |  [----]  |
      +----------+
    -------+------- (Junc #2)
           |
    -------+------- (Junc #1)
           |
      [  START   ]
```

### lf_until_color(target_color, vmax)
Follow line until a specific color zone is detected.
- **`target_color`**: The color constant to stop at (e.g., `S.RED`, `S.BLUE`).
- **`vmax`**: Maximum speed (0-1000).

```yaml
Visual: |
      [  STOP  ]
      +--------+
      | [----] |
      | |  O | |  <-- Robot stops on target
      | [----] |
      +--------+
      ==========  (TARGET ZONE)
          |
          | (Line Follow)
          |
      +--------+
      | [----] |
      | |  O | |
      | [----] |
      +--------+
      [ START  ]
```

### detect_color_sequence(n_colors)
Drive and record a sequence of colors (e.g., mission instructions).
- **`n_colors`**: Number of color markers to read.
- **Returns**: A list of color constants detected in order.

---

## [4] ATTACHMENT CONTROL

### motor_run_until_stall(p, speed)
Grip or move arm until it hits an object/mechanical limit.
- **`p`**: The motor port (e.g., `C.PORT_C1`).
- **`speed`**: Power level (Positive or Negative depending on direction).

### motor_home(p)
Reset arm to 0 by driving it until it hits the mechanical stop.
- **`p`**: The motor port.

---

## [5] ALIGNMENT TOOLS

### wall_align(speed)
Square the robot against a wall to reset heading.
- **`speed`**: Backward speed (usually 200-300).

### align_to_wall_color(target_color)
Drive until both sensors detect a colored zone to ensure straight entry.
- **`target_color`**: The color constant to align with.
