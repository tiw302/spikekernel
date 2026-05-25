# WRO High-Performance Robotics Framework
### Engineering Whitepaper & System Documentation

**English** | [ภาษาไทย](README_TH.md)

---

## Motivation
This project was born out of a necessity for a high-performance, reliable, and engineering-centric framework for WRO. After finding a lack of open-source solutions that meet professional standards for concurrency and mathematical precision, I developed this framework to push the boundaries of what is possible on the LEGO SPIKE Prime platform.

---

## Table of Contents

| Concept | System Core | Navigation | Resources |
|---|---|---|---|
| [Philosophy](#navigation-theory--mathematics) | [kernel.py](#kernelpy-orchestrator) | [pid_lib.py](#pid_libpy-motion-engine) | [config.py](#configpy-parameters) |
| [Quick Start](#quick-start) | [setup.py](#setuppy-initialization) | [sensor_lib.py](#sensor_libpy-perception) | [API Reference](#api-reference) |
| [Architecture](#system-architecture) | | | |

---

## Quick Start

Mission logic should be written in `main.py` or within the `missions/` directory. You can easily chain non-blocking asynchronous commands to control the robot. Here is a basic example:

```python
import system.pid_lib as P

async def mission_1(odo, tm):
    # 1. Move straight 50 cm (with automatic smooth acceleration & deceleration)
    await P.straight(odo, distance_cm=50)

    # 2. Turn to an absolute heading of 90 degrees
    await P.turn(odo, target_heading=90)

    # 3. Start a background task to move an arm on port E to 45 degrees, 
    # while simultaneously driving forward 20 cm!
    tm.start(P.motor_to_angle(port='E', target_deg=45))
    await P.straight(odo, distance_cm=20)
```

---

## System Architecture
The framework is built on a **Decoupled Library-Kernel Model**, separating low-level hardware abstraction from high-level mission orchestration.

*   **main.py**: User-space mission script.
*   **system/**: Core mathematical and hardware libraries.
*   **sys.path Extension**: The `system/` directory is added to `sys.path` to allow seamless imports.

---

## Core Components

### kernel.py (Orchestrator)
The system's executive layer. It manages the asynchronous event loop, initializes the Task Manager, and maintains the background E-Stop listener.

```python
import kernel

async def mission(odo, tm):
    # your logic here
    pass

# kernel.run handles setup, try/except, and cleanup automatically
kernel.run(mission)
```

### pid_lib.py (Motion Engine)
The mathematical heart of the robot. Implements Odometry fusion, PID control, S-Curve profiles, and Pure Pursuit navigation.

```python
import pid_lib as P

# Precision straight movement with S-Curve acceleration
await P.straight(odo, distance_cm=50, speed=400)

# Gyro-controlled turn to absolute heading
await P.turn(odo, target_heading=90)
```

### sensor_lib.py (Perception)
Handles raw sensor data processing, including reflection normalization and line-centroid estimation.

```python
import sensor_lib as S

# Get weighted line error from dual sensors
error = S.LineEst.dual()

# Calibration utility
S.CAL.white(port=C.PORT_C1)
```

### config.py (Parameters)
Centralized configuration for ports, physical dimensions, and PID gains. Modify this file to adapt the framework to different robot designs.

```python
# >> wheel dimensions
WHEEL_DIAMETER_CM = 5.6
AXLE_TRACK_CM     = 12.5

# >> PID Gains
DRIVE_KP = 1.2
DRIVE_KD = 0.05
```

---

## API Reference

### 1. Drive & Navigation (`pid_lib.py`)

| Function | Parameters | Description |
|---|---|---|
| `straight` | `odo, dist_cm (cm), vmax` | Move straight using Gyro and Odometry feedback with S-Curve profiles. |
| `turn` | `odo, target_h (degrees), vmax` | Spin turn to an absolute heading using Gyro PID. |
| `pivot_turn` | `odo, target_h (degrees), side` | Pivot turn on one wheel (`side='left'` or `'right'`). |
| `swing_turn` | `odo, target_h (degrees), outer_speed, inner_ratio` | Smooth curve turn with differential wheel speeds. |
| `arc` | `odo, radius_cm (cm), angle_deg (degrees), vmax` | Drive along a geometric arc of a defined radius and angle. |
| `goto_xy` | `odo, tx (cm), ty (cm), vmax` | Drive directly to an absolute target coordinate (X, Y) on the field. |
| `follow_path` | `odo, waypoints, default_vmax, smooth` | Smooth path-following using Pure Pursuit (highly smooth). |
| `wall_align` | *None* | Force-align against a wall and reset Gyro/Odometry. |

### 2. Line Tracking & Perception (`pid_lib.py` & `sensor_lib.py`)

| Function | Module | Parameters | Description |
|---|---|---|---|
| `track_line` | `pid_lib` | `odo, dist_cm (cm), vmax, sensor_port, edge` | Line follow using 1 sensor (C1/C2) while updating Odometry (X,Y) in real-time. |
| `lf_pd` | `sensor_lib` | `dist_cm (cm), vmax, p` | Simple 1-sensor line follower (PD) by distance. |
| `lf_gyro` | `sensor_lib` | `dist_cm (cm), vmax, p` | 1-sensor line follower with Gyro assist for speed stabilization. |
| `lf_dual` | `sensor_lib` | `dist_cm (cm), vmax` | 2-sensor line follower (Centroid) by distance. |
| `lf_dual_gyro`| `sensor_lib` | `dist_cm (cm), vmax` | 2-sensor line follower + Gyro assist (fastest/most stable). |
| `lf_n_junctions`| `sensor_lib` | `n (count), vmax, mode` | Follow line and stop exactly at the N-th junction. |
| `until_line` | `sensor_lib` | `vmax, heading (degrees)` | Drive at a specific heading until a line is detected. |
| `center_on_line`| `sensor_lib` | `speed, p` | Fine adjustments to center on a line edge. |

### 3. Arm & Attachment Control (`pid_lib.py`)

| Function | Parameters | Description |
|---|---|---|
| `motor_home` | `port, speed` | Home attachment arm by stalling against physical stop, setting position to 0. |
| `motor_to_angle` | `port, target_deg (degrees), speed` | Move arm to absolute angle (e.g. 45 degrees) using PID. |
| `motor_run_until_stall` | `port, speed` | Run motor until stall (gripping/pressing). |
| `motor_run_time` | `port, speed, duration_ms (ms)` | Run motor for a specific time. |
| `straight_with_motor` | `odo, dist_cm (cm), vmax, arm_port, arm_target_deg (degrees)` | Run straight drive and move arm simultaneously. |

### 4. Task Management (`kernel.py`)

| Function | Usage | Description |
|---|---|---|
| `tm.start` | `tm.start(coro)` | Start a non-blocking background task (e.g., motor movement). |
| `tm.cancel_all` | `tm.cancel_all()` | Kill all running background tasks immediately. |

---

## Navigation Theory & Mathematics

### 1. Sensor Fusion Odometry (Kalman Filter)
To mitigate gyro drift and encoder slip, we implement a simplified **1D Kalman Filter** for heading estimation:
*   **Prediction:** $h_{t} = h_{t-1} + \omega \cdot \Delta t$
*   **Correction:** $K = \frac{P_{p}}{P_{p} + R}$; $h_{t} = h_{t} + K \cdot (z - h_{t})$

### 2. Path Following (Pure Pursuit)
Trajectory tracking uses **Pure Pursuit Geometry**:
*   **Curvature ($\kappa$):** Calculated as $\frac{2 \cdot \Delta y}{L_d^2}$, defining the arc to the target waypoint.
*   **Steering:** Differential wheel speeds are calculated via $V_{L,R} = V \cdot (1 \pm \kappa \cdot \frac{W}{2})$.

### 3. Motion Profiling (Sigmoid S-Curve)
Velocity is controlled via a **Logistic Sigmoid Function**:
$$v(p) = v_{min} + \frac{v_{max} - v_{min}}{1 + e^{-k(p - x_0)}}$$

---

## Concurrency & Safety

### Asynchronous Task Management
The system utilizes a custom `TaskManager` to handle concurrent hardware operations:
*   **Non-blocking IO:** Attachments (arms/grippers) operate on independent coroutines while the drive-base executes navigation logic.
*   **E-Stop Supervisor:** Monitors hub buttons to trigger immediate `sys.exit()` and motor braking upon detection of a safety violation.

## Performance Optimization
*   **Bytecode Pre-compilation:** Critical paths use `@micropython.native` for machine-code execution speeds.
*   **Memory Management:** Explicit `gc.collect()` and buffered logging minimize heap fragmentation.

---

## License
This project is licensed under the [GNU General Public License v2.0](LICENSE) - see the [LICENSE](LICENSE) file for details.

**Engineered for Victory.**
**World Robot Olympiad Competition Framework**
