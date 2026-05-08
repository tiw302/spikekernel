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

### straight(odo, dist_cm)
Drive Forward (+) or Backward (-) in a straight line.
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
Rotate the robot on its center axis to a specific angle.
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
      |  |[X] |  |  <-- Left wheel stays still
      |  [----]  |
      +----------+
```

---

## [3] ADVANCED SENSOR MOTION (sensor_lib)

### lf_n_junctions(n, vmax)
Follow line and stop exactly at the Nth junction (Intersection).
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
Drive and record a sequence of colors (e.g., sorting).
```yaml
Visual: |
    [START] --> [RED] --> [BLUE] --> [GREEN]
      |          |         |          |
    [---]      [---]     [---]      [---]
    | O | ---> | O | --->| O | ---> | O |
    [---]      [---]     [---]      [---]
```

---

## [4] ATTACHMENT CONTROL

### motor_run_until_stall(p, speed)
Grip or move arm until it hits an object/mechanical limit.
```yaml
Visual: |
    [ OPEN ]           [ CLOSED / STALL ]
     \        /             |        |
      \      /    ---->     | [OBJ]  |
       [----]               [--------]
```

### motor_home(p)
Reset arm to 0 by driving it until it hits the mechanical stop.

---

## [5] ALIGNMENT TOOLS

### wall_align(speed)
Square the robot against a wall to reset heading.
```yaml
Visual: |
    ======================== [ WALL ]
      +----------+
      |  [----]  | <--- (Push / Square)
      |  |  O |  |
      |  [----]  |
      +----------+
```

### align_to_wall_color(target_color)
Drive until both sensors detect a colored zone to ensure straight entry.
```yaml
Visual: |
      +--------+
      | [----] |
      | |  O | |  <-- Both Sensors on Color
      | [----] |
      +--------+
    ============== (COLOR WALL)
          ^
          | (Approaching)
```
