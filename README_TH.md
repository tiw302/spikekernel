# WRO High-Performance Robotics Framework
### เอกสารอ้างอิงทางวิศวกรรมและระบบควบคุม

[English Edition](README.md) | **ภาษาไทย**

---

## แรงจูงใจในการพัฒนา (Motivation)
โปรเจกต์นี้เกิดขึ้นจากความต้องการเฟรมเวิร์กหุ่นยนต์สำหรับ WRO ที่มีสมรรถนะสูง เชื่อถือได้ และยึดตามหลักวิศวกรรม เนื่องจากในปัจจุบันยังขาดแคลนโอเพนซอร์สที่ตอบโจทย์มาตรฐานระดับมืออาชีพ ทั้งในด้านการทำงานแบบขนาน (Concurrency) และความแม่นยำทางคณิตศาสตร์ ผมจึงพัฒนาเฟรมเวิร์กนี้ขึ้นเพื่อก้าวข้ามขีดจำกัดของแพลตฟอร์ม LEGO SPIKE Prime

---

## สารบัญ (Table of Contents)

| แนวคิดหลัก | ระบบควบคุมกลาง | การเคลื่อนที่ | ข้อมูลอ้างอิง |
|---|---|---|---|
| [หลักการทางคณิตศาสตร์](#หลักการทางคณิตศาสตร์และทฤษฎีการเดินรถ) | [kernel.py](#kernelpy-ตัวจัดการระบบกลาง) | [pid_lib.py](#pid_libpy-เครื่องยนต์การเคลื่อนที่) | [config.py](#configpy-การตั้งค่าพารามิเตอร์) |
| [สถาปัตยกรรมระบบ](#สถาปัตยกรรมระบบ) | [setup.py](#setuppy-การตั้งค่าเริ่มต้น) | [sensor_lib.py](#sensor_libpy-การรับรู้และเซนเซอร์) | [ข้อมูลอ้างอิง API](#ข้อมูลอ้างอิง-api) |

---

## สถาปัตยกรรมระบบ (System Architecture)
เฟรมเวิร์กนี้ถูกออกแบบมาในรูปแบบ **Decoupled Library-Kernel Model** ซึ่งแยกส่วนการทำงานระหว่างฮาร์ดแวร์ระดับล่าง (Low-level) และส่วนการจัดการภารกิจระดับสูง (High-level) ออกจากกันอย่างชัดเจน

*   **main.py**: สคริปต์สำหรับเขียนภารกิจของผู้ใช้ (User-space)
*   **system/**: โฟลเดอร์เก็บไลบรารีทางคณิตศาสตร์และระบบควบคุมหลัก
*   **sys.path Extension**: มีการเพิ่มโฟลเดอร์ `system/` เข้าไปในเส้นทางการค้นหาของ Python เพื่อให้สามารถ Import ไฟล์ได้ง่าย

---

## องค์ประกอบหลัก (Core Components)

### kernel.py (ตัวจัดการระบบกลาง)
ทำหน้าที่เป็นส่วนบริหารจัดการระบบ (Executive Layer) คอยควบคุม Event Loop แบบ Asynchronous, เริ่มการทำงานของ Task Manager และคอยตรวจจับปุ่มหยุดฉุกเฉิน (E-Stop) ในเบื้องหลัง

```python
import kernel

async def mission(odo, tm):
    # เขียนลำดับภารกิจที่นี่
    pass

# kernel.run จะจัดการเรื่อง setup และการเคลียร์ทรัพยากรให้อัตโนมัติ
kernel.run(mission)
```

### pid_lib.py (เครื่องยนต์การเคลื่อนที่)
หัวใจทางคณิตศาสตร์ของหุ่นยนต์ รวบรวมอัลกอริทึม Odometry, การควบคุมแบบ PID, การเคลื่อนที่แบบ S-Curve และการนำทางแบบ Pure Pursuit

```python
import pid_lib as P

# การเดินตรงอย่างแม่นยำด้วย S-Curve
await P.straight(odo, distance_cm=50, speed=400)

# การเลี้ยวตามองศาจีโร (Absolute Heading)
await P.turn(odo, target_heading=90)
```

### sensor_lib.py (การรับรู้และเซนเซอร์)
จัดการการประมวลผลข้อมูลดิบจากเซนเซอร์ รวมถึงการทำ Calibration และการหาจุดศูนย์กลางของเส้น (Line-centroid estimation)

```python
import sensor_lib as S

# หาค่าความผิดพลาดจากเส้นโดยใช้เซนเซอร์คู่
error = S.LineEst.dual()

# เครื่องมือสำหรับการ Calibrate
S.CAL.white(port=C.PORT_C1)
```

### config.py (การตั้งค่าพารามิเตอร์)
แหล่งรวบรวมการตั้งค่าพอร์ต, ขนาดทางกายภาพของหุ่นยนต์ และค่า Gain ของ PID สามารถแก้ไขไฟล์นี้เพื่อปรับเปลี่ยนให้เข้ากับหุ่นยนต์รูปแบบต่างๆ

```python
# >> wheel dimensions
WHEEL_DIAMETER_CM = 5.6
AXLE_TRACK_CM     = 12.5

# >> PID Gains
DRIVE_KP = 1.2
DRIVE_KD = 0.05
```

---

## ข้อมูลอ้างอิง API (API Reference)

### คำสั่งการเคลื่อนที่พื้นฐาน (`pid_lib.py`)

| ฟังก์ชัน | พารามิเตอร์ | คำอธิบาย |
|---|---|---|
| `straight` | `odo, dist, speed` | เดินตรงโดยใช้ Odometry และระบบ S-Curve |
| `turn` | `odo, heading` | หมุนตัวไปยังองศาที่กำหนดด้วย Gyro PID |
| `arc` | `odo, r, angle` | เลี้ยวโค้งตามรัศมีที่กำหนดอย่างนุ่มนวล |
| `follow_line` | `odo, dist, kp, kd` | เดินตามเส้นโดยใช้เซนเซอร์คู่พร้อมการแก้ค่า PID |

### การจัดการงานขนาน (`kernel.py` / `pid_lib.py`)

| ฟังก์ชัน | การใช้งาน | คำอธิบาย |
|---|---|---|
| `tm.start` | `tm.start(coro)` | เริ่มงานเบื้องหลังแบบขนาน (เช่น การขยับแขนขณะวิ่ง) |
| `tm.cancel_all` | `tm.cancel_all()` | สั่งหยุดงานเบื้องหลังทั้งหมดทันที |

---

## หลักการทางคณิตศาสตร์และทฤษฎีการเดินรถ

### 1. Sensor Fusion Odometry (Kalman Filter)
เพื่อลดการสะสมความผิดพลาดของ Gyro และการลื่นไถลของล้อ เราได้นำ **1D Kalman Filter** มาใช้ในการประมาณค่าทิศทาง:
*   **Prediction:** $h_{t} = h_{t-1} + \omega \cdot \Delta t$
*   **Correction:** $K = \frac{P_{p}}{P_{p} + R}$; $h_{t} = h_{t} + K \cdot (z - h_{t})$

### 2. การเดินตามเส้นทาง (Pure Pursuit)
การเคลื่อนที่ตามพิกัดจะใช้เรขาคณิตแบบ **Pure Pursuit**:
*   **Curvature ($\kappa$):** คำนวณจาก $\frac{2 \cdot \Delta y}{L_d^2}$ เพื่อหาความโค้งของวงเลี้ยวไปยังจุดหมาย

### 3. Motion Profiling (Sigmoid S-Curve)
ความเร็วจะถูกควบคุมผ่านฟังก์ชัน **Logistic Sigmoid**:
$$v(p) = v_{min} + \frac{v_{max} - v_{min}}{1 + e^{-k(p - x_0)}}$$

---

## การทำงานแบบขนานและความปลอดภัย

### การจัดการงานแบบ Asynchronous
ระบบใช้ `TaskManager` เพื่อจัดการการทำงานของฮาร์ดแวร์หลายอย่างพร้อมกัน:
*   **Non-blocking IO:** อุปกรณ์เสริม (แขน/มือจับ) สามารถทำงานแยกกันได้ในขณะที่ฐานล้อกำลังวิ่งตามเส้นทาง
*   **E-Stop Supervisor:** ตรวจสอบปุ่มบนฮับเพื่อสั่งหยุดการทำงานและเบรกมอเตอร์ทันทีหากพบเหตุฉุกเฉิน

## การเพิ่มประสิทธิภาพ (Performance Optimization)
*   **Bytecode Pre-compilation:** ส่วนสำคัญของโค้ดจะถูกตกแต่งด้วย `@micropython.native` เพื่อให้ทำงานในระดับ Machine Code
*   **Memory Management:** มีการเรียกใช้ `gc.collect()` และการทำบัฟเฟอร์ล็อกเพื่อลดปัญหา RAM เต็ม

---

## สัญญาอนุญาต (License)
โปรเจกต์นี้ใช้สัญญาอนุญาตแบบ [GNU General Public License v2.0](LICENSE) - ดูรายละเอียดเพิ่มเติมได้ในไฟล์ [LICENSE](LICENSE)

**Engineered for Victory.**
**World Robot Olympiad Competition Framework**
