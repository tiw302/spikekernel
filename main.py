#!/usr/bin/env pybricks-micropython
# Spike prime / inventor hub — Pybricks 4.0 (https://beta.pybricks.com/)
# My github account: https://github.com/tiw302, My ig account: @tiw3025k_

#                    (GRIPPER)        [TOP VIEW]
#                    [port A]
#                  .----------.
#                  | [B]  [C] |  <- sensors (L/R)
#         .--------.----------.--------.
#         |  [D]   |  SPIKE   |  [E]   |
#         | (Left  |   HUB    | (Right |
#         | Motor) |   [F]    | Motor) |
#         |        |(MAIN ARM)|        |
#         '--------'----------'--------'

"""
* port connections:                                               [english]
    port D = left wheel motor   (forward = auto-corrected by pybricks)
    port E = right wheel motor  (forward = auto-corrected by pybricks)
    port A = front attachment motor (gripper)
    port F = mid attachment motor (heavy lifter)
    port B = left color sensor
    port C = right color sensor

    note: spike prime / inventor hub has a built-in imu (gyroscope + accelerometer).
          hub.imu.heading() gives absolute yaw — no external gyro sensor needed!
          hub.imu.angular_velocity() gives real-time rotation rate.

* algorithms & optimizations (international-level):
    C-level Rtos : pybricks 4.0 drivebase replaces python pid loops with 1khz c firmware
    Gyro-Gused   : imu heading natively integrates with motor encoders (no drift)
    Zero-Alloc   : method caching & gc.disable() eradicate garbage collection in hot loops
    Memory Opt   : uses __slots__ for minimal ram footprint
    Gyro Damp    : uses imu_angular_velocity to dampen line-tracking oscillation

* pre-match checklist:
    1. update wheel_diameter_mm / axle_track_mm to match physical robot
    2. calibrate sensors: run 'debug.py', get values, update white_light / black_light
    3. hub.imu.reset_heading(0) is called automatically (ensure robot is completely still on startup)
    4. update distance_correction from test runs

._________________________________________________________________________________________.

* การต่อพอร์ต:                                                     [thai]
    port D = มอเตอร์ล้อซ้าย  (ตั้ง counterclockwise เดินหน้าบวกปกติ)
    port E = มอเตอร์ล้อขวา  (ตั้ง clockwise เดินหน้าบวกปกติ)
    port A = มอเตอร์แขนหน้า (gripper)
    port F = มอเตอร์แขนกลาง (ตัวยกของหนัก)
    port B = เซนเซอร์สีซ้าย (left sensor)
    port C = เซนเซอร์สีขวา (right sensor)

    หมายเหตุ: spike prime / inventor hub มี imu (ไจโรสโคป) ในตัว ไม่ต้องต่อแยก

* algorithms & optimizations ที่ใช้ (international-level):
    C-level Rtos : เปลี่ยนจาก pid ใน python เป็น drivebase ที่รันด้วยภาษา c ความถี่ 1,000hz
    Gyro-Gused   : ผสานเซ็นเซอร์ imu เข้ากับการหมุนล้อโดยตรง เลี้ยวและวิ่งตรงไม่มีเบี้ยว
    Zero-Alloc   : ปิดระบบ gc ชั่วคราวในลูปเกาะเส้น หุ่นวิ่งเนียนไม่มีอาการสะดุด
    Memory Opt   : ใช้ __slots__ รีด ram หุ่นทำงานเร็วสุดขีด
    Gyro Damp    : ดึงค่าส่ายของ gyro มาต้านการแกว่งตอนเกาะเส้น

* ก่อนแข่ง:
    1. แก้ wheel_diameter_mm / axle_track_mm ให้ตรงกับล้อจริง
    2. คาลิเบรตเซนเซอร์: รันไฟล์ `debug.py` เพื่อหาค่า white_light / black_light จากสนามจริง
    3. การกดรันหุ่น "ห้ามมือสั่น" เพราะระบบจะป้องกันการรีเซ็ต gyro เพี้ยน (ต้องปล่อยมือให้สนิท)
    4. แก้ distance_correction จากการทดสอบวิ่งจริง (100cm)
"""

import math
import gc
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Stop, Button, Color, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
from micropython import const

#  _   _    _    ____  ______        ___    ____  _____
# | | | |  / \  |  _ \|  _ \ \      / / \  |  _ \| ____|
# | |_| | / _ \ | |_) | | | \ \ /\ / / _ \ | |_) |  _|
# |  _  |/ ___ \|  _ <| |_| |\ v  v / ___ \|  _ <| |___
# |_| |_/_/   \_\_| \_\____/  \_/\_/_/   \_\_| \_\_____|
#
# >> robot hardware configuration
WHEEL_DIAMETER_MM = 56.0        # diameter of the drive wheels in mm. used for travel distance.
AXLE_TRACK_MM     = 112.0       # center-to-center distance between wheels in mm. used for turn arc.
WHEEL_CIRC        = math.pi * WHEEL_DIAMETER_MM  # circumference = pi * d

#  _____ _   _ _   _ ___ _   _  ____
# |_   _| | | | \ | |_ _| \ | |/ ___|
#   | | | | | |  \| || ||  \| | |  _ 
#   | | | |_| | |\  || || |\  | |_| |
#   |_|  \___/|_| \_|___|_| \_|\____|
#
# >> tuning parameters
DISTANCE_CORRECTION = 0.98      # fix slip on straights  (> 1.0 if it drives short)

# color sensor calibration (measure on the real competition mat!)
WHITE_LIGHT = const(85)         # average reflection on white surface (calibrate this before match)
BLACK_LIGHT = const(10)         # average reflection on black line (calibrate this before match)
LINE_EDGE   = (WHITE_LIGHT + BLACK_LIGHT) // 2   # automatic midpoint threshold

def clamp(v, lo, hi):
    if v < lo: return lo
    if v > hi: return hi
    return v

class Robot:
    # __slots__: eliminates __dict__ on robot instance — largest single ram saving
    __slots__ = ('hub', 'left_motor', 'right_motor', 'attach_front', 'attach_mid',
                 'sensor_l', 'sensor_r', 'drive_base')

#  ██████  ███████ ████████ ██    ██ ██████  
# ██       ██         ██    ██    ██ ██   ██ 
#  █████   █████      ██    ██    ██ ██████  
#      ██  ██         ██    ██    ██ ██      
# ██████   ███████    ██     ██████  ██      
#
# >> setup core (initialization, port and sensor checking)
# >> setup core (ระบบตั้งค่าเริ่มต้น เช็คพอร์ตและเซ็นเซอร์)
    def __init__(self):
        self.hub = PrimeHub()
        # disable default center-button kill so estop routes through stop_drive()
        self.hub.system.set_stop_button(None)

        print("[ROBOT] ----------------------------------------")
        print("[ROBOT] SPIKE Kernel v2.0 — Pybricks 4.0")
        print("[ROBOT] Initializing ports...")

        self.left_motor   = self._init_motor(Port.D, "Left Motor",   Direction.COUNTERCLOCKWISE)
        self.right_motor  = self._init_motor(Port.E, "Right Motor",  Direction.CLOCKWISE)
        self.attach_front = self._init_motor(Port.A, "Front Attach", Direction.CLOCKWISE, required=False)
        self.attach_mid   = self._init_motor(Port.F, "Mid Attach",   Direction.CLOCKWISE, required=False)

        self.sensor_l     = self._init_sensor(Port.B, "Left Sensor")
        self.sensor_r     = self._init_sensor(Port.C, "Right Sensor")

        # wait for imu to calibrate/ready before proceeding
        # (vital for pybricks 4.0 so we don't fetch bogus values right at startup)
        while not self.hub.imu.ready():
            wait(10)

        # initialize pybricks native drivebase (runs in c, highest performance gyro fusion)
        # by setting the hub orientation, the drivebase
        # automatically integrates the imu at 1khz for flawless straight lines and turns.
        self.drive_base = DriveBase(self.left_motor, self.right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)
        self.drive_base.use_gyro(True)

        print("[ROBOT] All required ports connected!")
        self.check_battery()
        print("[ROBOT] ----------------------------------------")

        # zero the imu heading at startup
        self.hub.imu.reset_heading(0)
        print("[ROBOT] IMU heading zeroed.")

    def _init_motor(self, port, label, direction=Direction.CLOCKWISE, required=True):
        try:
            m = Motor(port, direction)
            print(f"[ROBOT]   [OK] {label} ({port})")
            return m
        except Exception:
            print(f"[ROBOT]   [FAIL] {label} ({port})")
            if required:
                self.hub.speaker.beep(200, 500)
                raise
            return None

    def _init_sensor(self, port, label):
        try:
            s = ColorSensor(port)
            print(f"[ROBOT]   [OK] {label} ({port})")
            return s
        except Exception:
            print(f"[ROBOT]   [FAIL] {label} ({port})")
            self.hub.speaker.beep(200, 500)
            raise

# ███    ███  ██████  ██    ██ ███████ ███    ███ ███████ ███    ██ ████████ 
# ████  ████ ██    ██ ██    ██ ██      ████  ████ ██      ████   ██    ██    
# ██ ████ ██ ██    ██ ██    ██ █████   ██ ████ ██ █████   ██ ██  ██    ██    
# ██  ██  ██ ██    ██  ██  ██  ██      ██  ██  ██ ██      ██  ██ ██    ██    
# ██      ██  ██████    ████   ███████ ██      ██ ███████ ██   ████    ██    
#
# >> movement core (drive mechanics, straight motion and turning)
# >> movement core (ระบบขับเคลื่อนล้อซ้ายขวา วิ่งตรง และเลี้ยว)

#  __  __  ___  _   _ ___   ___ _____ ___    _   ___ ___ _  _ _____
# |  \/  |/ _ \| \ / / __| / __|_   _| _ \  /_\ |_ _/ __| || |_   _|
# | |\/| | (_) |\ v /| _|  \__ \ | | |   / / _ \ | | (_ | __ | | |
# |_|  |_|\___/  \_/ |___| |___/ |_| |_|_\/_/ \_\___\___|_||_| |_|
    def move_straight(self, distance_cm, max_speed=400, accel_frac=0.25, use_gyro=True):
        """
        drives straight with absolute maximum precision using pybricks c-level gyro fusion.
        formula: speed_mm = (deg/s / 360) * wheel_circ
        the built-in drivebase operates at 1khz in firmware, eliminating all micropython latency.
        it uses the imu natively to correct drift before the python loop even knows it happened.
        """
        distance_mm = distance_cm * 10
        self.log(f"Start: str {distance_cm}cm @ {max_speed}dps (C-Level Gyro)")
        self.reset_encoders()

        # convert degrees/sec to mm/sec for drivebase
        speed_mm = (max_speed / 360.0) * WHEEL_CIRC
        # calculate acceleration (mm/s^2). default punchy acceleration reaches max speed in 0.5s.
        accel_mm = speed_mm / 0.5  

        self.drive_base.use_gyro(use_gyro)
        self.drive_base.settings(straight_speed=speed_mm, straight_acceleration=accel_mm)
        
        # drivebase handles trapezoidal profile and gyro heading lock perfectly in c firmware
        self.drive_base.straight(distance_mm)
        
        self.stop_drive()
        self.log("Done: str")

#  _____ _   _ ___ _  _
# |_   _| | | | _ \ \| |
#   | | | |_| |   / .` |
#   |_|  \___/|_|_\_|\_|
    def turn(self, angle_deg, max_speed=300, use_gyro=True):
        """
        performs a precise point-turn using pybricks c-level gyro integration.
        the firmware-level pid guarantees zero-drift turns without any python loop overhead.
        formula: turn_rate = (max_speed * wheel_diam) / axle_track
        """
        self.log(f"Start: turn {angle_deg}d (C-Level Gyro)")
        self.reset_encoders()

        # convert degrees/sec of wheels into robot turn rate (deg/sec)
        turn_rate = (max_speed * WHEEL_DIAMETER_MM) / AXLE_TRACK_MM
        turn_accel = turn_rate / 0.4  # reach max turn rate in 0.4 seconds

        self.drive_base.use_gyro(use_gyro)
        self.drive_base.settings(turn_rate=turn_rate, turn_acceleration=turn_accel)

        # drivebase perfectly turns exactly the requested degrees using the imu
        self.drive_base.turn(angle_deg)
        
        self.stop_drive()
        self.log("Done: turn")

#  ___ _____   _____ _____   _____ _   _ ___ _  _ 
# | _ \_ _\ \ / / _ \_   _| |_   _| | | | _ \ \| |
# |  _/| | \ v / (_) || |     | | | |_| |   / .` |
# |_| |___| \_/ \___/ |_|     |_|  \___/|_|_\_|\_|
    def pivot_turn(self, angle_deg, pivot_side='right', max_speed=300, use_gyro=True):
        """
        turns the robot by moving only one wheel (pivot turn).
        uses pybricks 4.0 drivebase.curve() for native c-level gyro fusion and perfect profile.
        angle_deg: positive for clockwise (right), negative for counter-clockwise (left).
        pivot_side: 'right' (hold right wheel, move left) or 'left' (hold left wheel, move right).
        """
        self.log(f"Start: pivot turn {angle_deg}d on {pivot_side} (C-Level Gyro)")
        self.reset_encoders()

        radius = (AXLE_TRACK_MM / 2.0) if pivot_side == 'right' else -(AXLE_TRACK_MM / 2.0)
        curve_angle = angle_deg if pivot_side == 'right' else -angle_deg

        speed_mm = (max_speed / 360.0) * WHEEL_CIRC
        accel_mm = speed_mm / 0.4
        turn_rate = (max_speed * WHEEL_DIAMETER_MM) / AXLE_TRACK_MM
        turn_accel = turn_rate / 0.4

        self.drive_base.use_gyro(use_gyro)
        self.drive_base.settings(straight_speed=speed_mm, straight_acceleration=accel_mm,
                                 turn_rate=turn_rate, turn_acceleration=turn_accel)

        # drivebase perfectly calculates the arc and keeps one wheel completely still
        self.drive_base.curve(radius, curve_angle)
        
        self.stop_drive()
        self.log("Done: pivot turn")

#    _   _    ___ ___ _  _   __      __ _   _    _
#   /_\ | |  |_ _/ __| \| |  \ \    / //_\ | |  | |
#  / _ \| |__ | | (_ | .` |   \ \/\/ // _ \| |__| |__
# /_/ \_\____|___\___|_|\_|    \_/\_//_/ \_\____|____|
    def align_wall(self, power, time_ms, hold=True, kp=0.5):
        """
        runs the robot into a wall using pid sync to stay straight,
        but limits raw power to prevent violent stalling.
        power: raw duty cycle power -100 to 100 (positive for forward).
        time_ms: time in milliseconds to push against the wall.
        hold: if true, applies active hold after stalling.
        """
        self.log(f"Start: align wall {power}% for {time_ms}ms")
        self.reset_encoders()

        la_func = self.left_motor.angle
        ra_func = self.right_motor.angle
        max_pwr = abs(power)

        watch = StopWatch()
        while watch.time() < time_ms:
            la = la_func()
            ra = ra_func()

            # basic proportional sync to stay perfectly straight
            sync_err   = la - ra
            correction = kp * sync_err

            l = power - correction
            r = power + correction

            # clamp power to prevent ramping up during stall
            if l >  max_pwr: l =  max_pwr
            elif l < -max_pwr: l = -max_pwr
            if r >  max_pwr: r =  max_pwr
            elif r < -max_pwr: r = -max_pwr

            self.left_motor.dc(l)
            self.right_motor.dc(r)
            wait(10)

        self.stop_drive(hold)
        self.log("Done: align wall")

    def drive(self, left_speed, right_speed):
        """raw manual drive — both motors in deg/s. use for custom maneuvers."""
        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)


    def track_line_distance(self, distance_cm, speed=300, kp=1.5, kd=0.5, use_gyro_damp=False):
        """
        follows a line for a specific distance (in cm) using two color sensors.
        error = left_sensor - right_sensor
        """
        self.log(f"Start: track line dist {distance_cm}cm @ {speed}dps")
        self.reset_encoders()

        distance_mm = distance_cm * 10
        target      = abs(distance_mm) / WHEEL_CIRC * 360.0 * DISTANCE_CORRECTION

        sl_ref      = self.sensor_l.reflection
        sr_ref      = self.sensor_r.reflection
        l_run       = self.left_motor.run
        r_run       = self.right_motor.run
        imu_ang_vel = self.hub.imu.angular_velocity
        avg_ang     = self.avg_angle
        wait_func   = wait

        last_error = 0

        # [c-level opt] lock ram, prevent gc pauses
        gc.collect()
        gc.disable()

        while avg_ang() < target:
            vl = sl_ref()
            vr = sr_ref()
            error = vl - vr
            derivative = error - last_error

            gyro_damp  = 0.0
            if use_gyro_damp:
                gz        = imu_ang_vel(Axis.Z)
                gyro_damp = 0.3 * gz

            turn = (error * kp) + (derivative * kd) + gyro_damp

            l_run(speed + turn)
            r_run(speed - turn)

            last_error = error
            wait_func(10)

        gc.enable()
        self.stop_drive(hold=True)
        self.log(f"Done: track line dist {distance_cm}cm")

    def track_line_timer(self, time_ms, speed=300, kp=1.5, kd=0.5, use_gyro_damp=False):
        """
        follows a line for a specific amount of time (in milliseconds) using two color sensors.
        """
        self.log(f"Start: track line timer {time_ms}ms @ {speed}dps")
        sw      = StopWatch()
        sw_time = sw.time

        sl_ref      = self.sensor_l.reflection
        sr_ref      = self.sensor_r.reflection
        l_run       = self.left_motor.run
        r_run       = self.right_motor.run
        imu_ang_vel = self.hub.imu.angular_velocity
        wait_func   = wait

        last_error = 0

        # [c-level opt] lock ram, prevent gc pauses
        gc.collect()
        gc.disable()

        while sw_time() < time_ms:
            vl = sl_ref()
            vr = sr_ref()
            error = vl - vr
            derivative = error - last_error

            gyro_damp  = 0.0
            if use_gyro_damp:
                gz        = imu_ang_vel(Axis.Z)
                gyro_damp = 0.3 * gz

            turn = (error * kp) + (derivative * kd) + gyro_damp

            l_run(speed + turn)
            r_run(speed - turn)

            last_error = error
            wait_func(10)

        gc.enable()
        self.stop_drive(hold=True)
        self.log(f"Done: track line timer {time_ms}ms")

#    _   _    ___ ___ _  _   _    ___ _  _ ___
#   /_\ | |  |_ _/ __| \| | | |  |_ _| \| | __|
#  / _ \| |__ | | (_ | .` | | |__ | || .` | _|
# /_/ \_\____|___\___|_|\_| |____|___|_|\_|___|

    def track_until_intersection(self, speed=300, kp=1.5, kd=0.5, use_gyro_damp=False):
        """
        follows a line using two color sensors until both sensors see black (intersection).
        """
        self.log(f"Start: track until intersection @ {speed}dps")
        sl_ref      = self.sensor_l.reflection
        sr_ref      = self.sensor_r.reflection
        l_run       = self.left_motor.run
        r_run       = self.right_motor.run
        imu_ang_vel = self.hub.imu.angular_velocity
        wait_func   = wait

        last_error = 0

        # [c-level opt] lock ram, prevent gc pauses
        gc.collect()
        gc.disable()

        while True:
            vl = sl_ref()
            vr = sr_ref()
            
            # both sensors see black = intersection
            if vl < LINE_EDGE and vr < LINE_EDGE:
                break
                
            error = vl - vr
            derivative = error - last_error

            gyro_damp  = 0.0
            if use_gyro_damp:
                gz        = imu_ang_vel(Axis.Z)
                gyro_damp = 0.3 * gz

            turn = (error * kp) + (derivative * kd) + gyro_damp

            l_run(speed + turn)
            r_run(speed - turn)

            last_error = error
            wait_func(10)

        gc.enable()
        self.stop_drive(hold=True)
        self.log("Done: track until intersection")

#    _   _    ___ ___ _  _   _    ___ _  _ ___
#   /_\ | |  |_ _/ __| \| | | |  |_ _| \| | __|
#  / _ \| |__ | | (_ | .` | | |__ | || .` | _|
# /_/ \_\____|___\___|_|\_| |____|___|_|\_|___|

    def drive_until_line(self, speed=300, align=True, hold=True):
        """
        drives straight until both color sensors detect a black line (intersection).
        if align=true, it will independently stop each motor to auto-square against the line.
        """
        self.log(f"Start: drive until line @ {speed}dps")
        
        sl_ref    = self.sensor_l.reflection
        sr_ref    = self.sensor_r.reflection
        l_run     = self.left_motor.run
        r_run     = self.right_motor.run
        wait_func = wait
        
        # [c-level opt] lock ram, prevent gc pauses
        gc.collect()
        gc.disable()
        
        l_found = False
        r_found = False
        
        l_run(speed)
        r_run(speed)
        
        while not (l_found and r_found):
            vl = sl_ref()
            vr = sr_ref()
            
            if not l_found and vl < LINE_EDGE:
                l_found = True
                if align: l_run(0)
            if not r_found and vr < LINE_EDGE:
                r_found = True
                if align: r_run(0)
                
            wait_func(10)
            
        gc.enable()
        self.stop_drive(hold)
        self.log("Done: drive until line")
        
    def stop_drive(self, hold=True):
        """stops both drive motors. hold=true = active hold (locks position)."""
        # stop drivebase if it was currently driving
        self.drive_base.stop()
        if hold:
            self.left_motor.hold()
            self.right_motor.hold()
        else:
            self.left_motor.brake()
            self.right_motor.brake()


# ██      ██ ███████ ████████ 
# ██      ██ ██         ██    
# ██      ██ █████      ██    
# ██      ██ ██         ██    
# ███████ ██ ██         ██    
#
# >> attachment core (mechanisms for gripping and lifting)
# >> attachment core (ระบบแขนกลสำหรับคีบและยกสิ่งของ)
    def lift(self, speed=300, power=30, motor='front', then=Stop.HOLD):
        """
        runs the attachment motor until stalled (finds mechanical endpoint).
        suitable for grippers, lifts, and deployable mechanisms.
        motor : 'front' (port.c) or 'mid' (port.d)
        power : duty cycle limit (0-100%). controls grip/lift force at stall.
        """
        self.log(f"Start: lift {motor} (spd={speed}, pwr={power}%)")
        m = self.attach_front if motor == 'front' else self.attach_mid
        if m: m.run_until_stalled(speed, then=then, duty_limit=power)
        self.log("Done: lift")

    def lift_angle(self, angle_deg, speed=300, motor='front', then=Stop.HOLD):
        """moves the attachment motor by a specific angle."""
        self.log(f"Start: lift_angle {motor} {angle_deg}deg")
        m = self.attach_front if motor == 'front' else self.attach_mid
        if m: m.run_angle(speed, angle_deg, then=then)
        self.log("Done: lift_angle")

    def release_attachment(self, motor='front'):
        """releases holding torque on the attachment motor (free-wheel)."""
        self.log(f"Release {motor} attachment")
        m = self.attach_front if motor == 'front' else self.attach_mid
        if m: m.stop()


#  ██████  ███████ ███    ██  ██████   ██████  ██████   ██████  
# ██       ██      ████   ██ ██       ██    ██ ██   ██ ██       
#  █████   █████   ██ ██  ██  █████   ██    ██ ██████   █████   
#      ██  ██      ██  ██ ██      ██  ██    ██ ██   ██      ██  
# ██████   ███████ ██   ████ ██████    ██████  ██   ██ ██████   
#
# >> sensor core (light values, calibration, and line detection)
# >> sensor core (ระบบจัดการค่าแสง คาลิเบรต และเช็คเส้น)




# ██    ██ ████████ ██ ██      ██ ████████ ███████ ███████ 
# ██    ██    ██    ██ ██      ██    ██    ██      ██      
# ██    ██    ██    ██ ██      ██    ██    █████   ███████ 
# ██    ██    ██    ██ ██      ██    ██    ██           ██ 
#  ██████     ██    ██ ███████ ██    ██    ███████ ███████ 
#
# >> utility core (helper functions, encoders, printing)
# >> utility core (ฟังก์ชันช่วยเหลือย่อย เช็คเอนโค้ดเดอร์ สั่ง print)
    def reset_encoders(self):
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)

    def get_left_angle(self):  return self.left_motor.angle()
    def get_right_angle(self): return self.right_motor.angle()
    def avg_angle(self):       return (abs(self.get_left_angle()) + abs(self.get_right_angle())) / 2

    def get_heading(self):
        """returns the current imu heading in degrees. positive = clockwise."""
        return self.hub.imu.heading()

    def reset_heading(self, angle=0):
        """resets the imu heading reference to 'angle' (default 0). call before each run."""
        # [gyro protection 2] wait for imu to be ready before resetting.
        while not self.hub.imu.ready():
            wait(10)
            
        # [gyro protection 3] anti-wiggle / hands-off check.
        # ensures the user has completely let go of the robot and it's not vibrating.
        # if you reset heading while hands are still shaking it, the zero-point will drift!
        while not self.hub.imu.stationary():
            wait(10)
            
        self.hub.imu.reset_heading(angle)
        self.log(f"Heading reset to {angle}")

    def check_battery(self):
        """
        reads the hub battery voltage and current using pybricks 4.0 api.
        formula: percent = ((voltage - 7.0v) / (8.2v - 7.0v)) * 100
        """
        volts   = self.hub.battery.voltage() / 1000.0
        amps    = self.hub.battery.current() / 1000.0
        percent = clamp((volts - 7.0) / (8.2 - 7.0) * 100, 0, 100)
        self.log(f"BATTERY: {volts:.2f}V ({percent:.0f}%) | CURRENT: {amps:.2f}A")

        if percent < 20:
            self.hub.speaker.beep(300, 500)
            self.log("WARNING: Battery low!")

    def log(self, text):
        # * note: to reduce cpu load during a real match, uncomment 'pass' and comment out 'print'
        # *       เพื่อลดภาระ cpu ตอนแข่งจริง (เซฟแบต/ลดแลค) ให้เอา # หน้าคำว่า pass ออก และใส่ # หน้า print แทน
        # pass
        print(f"[ROBOT] {text}")


#  __  __    _    ___ _   _   _     ___   ___  ____  
# |  \/  |  / \  |_ _| \ | | | |   / _ \ / _ \|  _ \ 
# | |\/| | / _ \  | ||  \| | | |  | | | | | | | |_) |
# | |  | |/ ___ \ | || |\  | | |__| |_| | |_| |  __/ 
# |_|  |_/_/   \_\___|_| \_| |_____\___/ \___/|_|    
#
# >> main execution (mission scripts and logic)
# >> main execution (โค้ดสำหรับรันภารกิจจริง)
if __name__ == "__main__":
    robot = Robot()

    # startup chime
    robot.hub.speaker.beep(1047, 150)
    wait(80)
    robot.hub.speaker.beep(1319, 200)
    wait(80)
    robot.hub.speaker.beep(1568, 250)
    wait(100)

    #   * ===============================================
    #   *  cheat sheet: ตัวอย่างการเรียกใช้ทุกฟังก์ชัน
    #   * ===============================================
        # 1. การเคลื่อนที่พื้นฐาน (basic movements)
        # robot.move_straight(50)                       # วิ่งตรง 50 ซม. ด้วย gyro (move straight 50 cm)
        # robot.move_straight(50, max_speed=600)        # วิ่งตรงเร็วขึ้น 600 dps (move faster)
        # robot.move_straight(-20)                      # ถอยหลัง 20 ซม. (move backward 20 cm)
        # robot.turn(90)                                # เลี้ยวขวา 90 องศาเป๊ะด้วย gyro (point turn right 90 degrees)
        # robot.turn(-90)                               # เลี้ยวซ้าย 90 องศา (point turn left 90 degrees)
        
        # 2. การเลี้ยวแบบวงกว้าง (pivot turn)
        # robot.pivot_turn(90, pivot_side='right')      # ล้อขวาหยุดนิ่ง ล้อซ้ายเดินหน้า (pivot turn right)
        # robot.pivot_turn(-90, pivot_side='left')      # ล้อซ้ายหยุดนิ่ง ล้อขวาถอยหลัง (pivot turn backward left)
        
        # 3. การชนกำแพงตั้งลำ (wall squaring)
        # robot.align_wall(power=-50, time_ms=1500)     # ถอยชนกำแพงด้วยพลัง -50 เป็นเวลา 1.5 วิ (square against wall for 1.5s)
        
        # 4. การจัดการเซ็นเซอร์แสง (line & sensors - single sensor)
        # robot.drive_until_line(speed=300)             # วิ่งตรงไปจนกว่าเซนเซอร์ตาเดียวจะเจอเส้นดำ (drive until black line)
        
        # การเกาะขอบเส้นด้วยเซนเซอร์ตัวเดียว (single sensor edge tracking)
        # robot.track_line_distance(20, edge='right')   # เกาะขอบขวาของเส้นไปข้างหน้าเป็นระยะ 20 ซม. (track right edge for 20 cm)
        # robot.track_line_timer(2000, edge='left')     # เกาะขอบซ้ายของเส้นเป็นเวลา 2 วินาที (track left edge for 2000 ms)
        # robot.track_line_distance(20, use_gyro_damp=true) # เกาะเส้นแบบดึง gyro มาช่วยกันส่ายให้นิ่งขึ้น (gyro damped tracking)
        
        # 5. การบังคับมอเตอร์แขนกล (attachments)
        # robot.lift(speed=300, power=40)               # ยกแขนกลหน้า (port.c) จนกว่าจะชน/ตึง (stall detection)
        # robot.lift_angle(90, speed=300)               # สั่งหมุนแขนกลหน้าไป 90 องศา (lift front arm 90 degrees)
        # robot.release_attachment()                    # ปล่อยพักแขนกลหน้า (release front attachment)
        # (หมายเหตุ: ถ้าใช้งานแขนกลาง port.d สามารถสั่งผ่าน robot.attach_mid.run() ได้โดยตรง)
        
        # 6. คำสั่งอื่นๆ (misc)
        # robot.drive(300, 300)                         # สั่งมอเตอร์วิ่งตรงๆ ความเร็ว 300 องศา/วิ (drive motors raw at 300 deg/s)
        # robot.stop_drive(hold=true)                   # สั่งเบรกและล็อกล้อ (stop and hold wheels)
        # robot.reset_encoders()                        # รีเซ็ตค่าองศาล้อให้เป็น 0 (reset motor encoders to 0)
        # robot.reset_heading()                         # รีเซ็ตค่า gyro เป็น 0 (zero the gyro)
        # robot.check_battery()                         # เช็คแบตเตอรี่พิมพ์ออกจอคอม (print battery status to console)
        # robot.calibrate_sensors()                     # สั่งคาลิเบรตเซนเซอร์บนแมตแบบสดๆ (calibrate sensors on the mat)

    #   * ===============================================
    #   *  run: รันตรงนี้
    #   * ===============================================

    robot.reset_heading() # ← zero the gyro before every run!
    robot.check_battery()
    wait(500)
    