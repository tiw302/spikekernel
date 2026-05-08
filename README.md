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
| [Architecture](#system-architecture) | [setup.py](#setuppy-initialization) | [sensor_lib.py](#sensor_libpy-perception) | [API Reference](#api-reference) |

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

### Movement Primitives (`pid_lib.py`)

| Function | Parameters | Description |
|---|---|---|
| `straight` | `odo, dist, speed` | Move straight using Odometry feedback and S-Curve profiles. |
| `turn` | `odo, heading` | Rotate to a specific absolute heading using Gyro PID. |
| `arc` | `odo, r, angle` | Perform a smooth arc turn with a defined radius. |
| `follow_line` | `odo, dist, kp, kd` | Follow a line using dual sensors with PID correction. |

### Task Management (`kernel.py` / `pid_lib.py`)

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
