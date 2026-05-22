"""
    config.py -- robot parameters and system constants.
    config.py -- พารามิเตอร์ของหุ่นยนต์และค่าคงที่ระบบ

    technical background:
    ---------------------
    this file contains all the "magic numbers" for the robot. 
    change these values to tune the robot's physical behavior 
    without touching the library logic.
    ไฟล์นี้ประกอบด้วย "เลขอาถรรพ์" ทั้งหมดของหุ่นยนต์
    แก้ไขค่าเหล่านี้เพื่อปรับพฤติกรรมทางกายภาพของหุ่นยนต์
    โดยไม่ต้องแก้ไขตรรกะในไลบรารี

    units:
    ------
    - distance: cm (เซนติเมตร)
    - speed: encoder power (0-1000) (กำลังมอเตอร์)
    - angle: degrees (องศา)
"""

from hub import port
from math import pi

# ██████   ██████  ██████  ████████ ███████ 
# ██   ██ ██    ██ ██   ██    ██    ██      
# ██████  ██    ██ ██████     ██    ███████ 
# ██      ██    ██ ██   ██    ██         ██ 
# ██       ██████  ██   ██    ██    ███████ 
#
# >>ports
# >>พอร์ตเชื่อมต่อ

PORT_L  = port.A
PORT_R  = port.B
PORT_C1 = port.C   # front color sensor (เซนเซอร์สีด้านหน้า)
PORT_C2 = port.D   # rear / second color sensor (เซนเซอร์สีด้านหลัง)

# ██   ██ ██   ██ ███████ ███████ ██      
# ██   ██ ██   ██ ██      ██      ██      
# ███████ █████   █████   █████   ██      
# ██   ██ ██  ██  ██      ██      ██      
# ██   ██ ██   ██ ███████ ███████ ███████ 
#
# >>wheel & physical
# >>ล้อและค่าทางกายภาพ

# measure carefully, everything derives from these two numbers
# วัดให้ละเอียด ทุกอย่างคำนวณมาจากสองค่านี้
WHEEL_DIAM_CM  = 5.6          # cm
WHEEL_BASE_CM  = 11.2         # cm, center-to-center contact patches (ระยะห่างล้อ)

# derived -- do not touch
# ค่าคำนวณอัตโนมัติ - ห้ามแก้ไข
WHEEL_CIRC_CM  = WHEEL_DIAM_CM * pi
CM_PER_COUNT   = WHEEL_CIRC_CM / 1440.0   # cm per encoder count (เซนติเมตรต่อหนึ่งติ๊ก)
COUNT_PER_CM   = 1440.0 / WHEEL_CIRC_CM   # encoder counts per cm (จำนวนติ๊กต่อหนึ่งเซนติเมตร)

#  ██████  ██    ██ ██████   ██████  
# ██       ██    ██ ██   ██ ██    ██ 
# ██   ███  ██  ██  ██████  ██    ██ 
# ██    ██   ████   ██   ██ ██    ██ 
#  ██████     ██    ██   ██  ██████  
#
# >>gyro & control
# >>ไจโรและการควบคุม

GYRO_SCALE     = 1.000        # spin 3600 deg, set = 3600 / reported_val (ค่าปรับสเกลไจโร)
HZ             = 100          # hz, gives dt = 0.01s (ความถี่ในการวนลูป)

# ███████ ██████  ███████ ███████ ██████  
# ██      ██   ██ ██      ██      ██   ██ 
# ███████ ██████  █████   █████   ██   ██ 
#      ██ ██      ██      ██      ██   ██ 
# ███████ ██      ███████ ███████ ██████  
#
# >>speed limits (0-1000)
# >>ขีดจำกัดความเร็ว (0-1000)

VMAX           = 600          # max speed (ความเร็วสูงสุด)
VTURN          = 350          # turning speed (ความเร็วเลี้ยว)
VMIN           = 90           # min speed (ความเร็วต่ำสุด)

#  ██████  ██████   ██████  ███████ ███████ 
# ██       ██   ██ ██    ██ ██      ██      
# ██   ███ ██████  ██    ██ █████   ███████ 
# ██    ██ ██   ██ ██    ██ ██           ██ 
#  ██████  ██   ██  ██████  ███████ ███████ 
#
# >>motion gains
# >>ค่า Gain การเคลื่อนที่

# straight -- cte controller
# วิ่งตรง -- ตัวควบคุมเส้นทาง (CTE)
CTE_KP         = 3.5
CTE_KI         = 0.01
CTE_KPH        = 2.5
CTE_KDR        = 0.8

# turn -- pd
# เลี้ยว -- ตัวควบคุม PD (Tuned for Hub 3)
TURN_KP        = 8.0          # ลด P ลงนิดหน่อยเพื่อไม่ให้กระชาก
TURN_KD        = 5.0          # เพิ่ม D เพื่อให้เบรกนิ่งขึ้น
TURN_ERR_TOL   = 0.7          # deg (ความคลาดเคลื่อนที่ยอมรับได้)
TURN_RATE_TOL  = 2.5          # deg/s (ความเร็วในการหมุนที่ยอมรับได้)
TURN_SETTLE    = 15           # loops (จำนวนลูปเพื่อยืนยันการนิ่ง)

# เลี้ยวแบบล้อเดียว / เลี้ยวแบบหมุน (Tuned for Hub 3)
PIVOT_KP       = 6.5
PIVOT_KD       = 3.5

# velocity pid (low level)
# PID ควบคุมความเร็ว (ระดับต่ำ)
VEL_KP         = 0.9
VEL_KI         = 0.06
VEL_KD         = 0.008

# ██   ██ ███████ 
# ██  ██  ██      
# █████   █████   
# ██  ██  ██      
# ██   ██ ██      
#
# >>kalman filter
# >>ฟิลเตอร์คัลมาน

KF_Q           = 0.001        # process noise (สัญญาณรบกวนระบบ)
KF_R_STILL     = 0.4          # measurement noise when still (ขณะนิ่ง)
KF_R_MOVING    = 7.0          # measurement noise when moving (ขณะเคลื่อนที่)

# ██   ██  █████  ██████  ██████  ██   ██  █████  ██████  ███████ 
# ██   ██ ██   ██ ██   ██ ██   ██ ██   ██ ██   ██ ██   ██ ██      
# ███████ ███████ ██████  ██   ██ ███████ ███████ ██████  █████   
# ██   ██ ██   ██ ██   ██ ██   ██ ██   ██ ██   ██ ██   ██ ██      
# ██   ██ ██   ██ ██   ██ ██████  ██   ██ ██   ██ ██   ██ ███████ 
#
# >>hardware compensation
# >>การชดเชยฮาร์ดแวร์

# backlash (encoder counts)
BL_L           = 8
BL_R           = 8

# deadband (power units)
DB_L           = 85
DB_R           = 85

# ███████ ████████  ██████  ██████  
# ██         ██    ██    ██ ██   ██ 
# ███████    ██    ██    ██ ██████  
#      ██    ██    ██    ██ ██      
# ███████    ██     ██████  ██      
#
# >>stop & creep logic
# >>การหยุดและการคลานเข้าหาเป้าหมาย

# predictive stop
# การหยุดแบบคำนวณล่วงหน้า
DECEL_CPS2     = 120.0        # cm/s² -- measure by braking (ความหน่วงตอนเบรก)
DECEL_SAFETY   = 1.15

# micro-correction final approach
# การคลานเพื่อปรับตำแหน่งในระยะสุดท้าย
CREEP_CM       = 4.0          # cm from target to start creep (ระยะเริ่มคลาน)
CREEP_PWR      = 85           # power during creep (กำลังตอนคลาน)
CREEP_VEL_THR  = 1.5          # cm/s (ความเร็วที่ยอมรับว่าหยุดแล้ว)
POS_TOL_CM     = 0.5          # cm stop tolerance (ความแม่นยำระยะหยุด)

# ██████   █████  ████████ ██   ██ 
# ██   ██ ██   ██    ██    ██   ██ 
# ██████  ███████    ██    ███████ 
# ██      ██   ██    ██    ██   ██ 
# ██      ██   ██    ██    ██   ██ 
#
# >>path following
# >>การเดินตามเส้นทาง

# s-curve shape
# รูปทรงโค้งตัว S (การเร่ง/ผ่อน)
SC_K_ACC       = 12.0
SC_K_DEC       = 10.0

# pure pursuit
# การเดินตามจุดอ้างอิงบนเส้นทาง (Pure Pursuit)
PP_LD_CM       = 12.0         # cm lookahead (ระยะมองไปด้านหน้า)
PP_LD_MIN_CM   = 6.0
PP_LD_MAX_CM   = 20.0

# ███████ ███████ ███    ██ ███████  ██████  ██████  ███████ 
# ██      ██      ████   ██ ██      ██    ██ ██   ██ ██      
# ███████ █████   ██ ██  ██ ███████ ██    ██ ██████  █████   
#      ██ ██      ██  ██ ██      ██ ██    ██ ██   ██ ██      
# ███████ ███████ ██   ████ ███████  ██████  ██   ██ ███████ 
#
# >>sensors
# >>เซนเซอร์

# stall
# การตรวจสอบมอเตอร์ติดขัด
STALL_PWR      = 150          # threshold power
STALL_ENC      = 3            # counts/loop threshold
STALL_LOOPS    = 20           # duration

# wall align
# การจัดระเบียบหุ่นยนต์เข้ากับผนัง
WA_SPEED       = 200
WA_MAX_MS      = 1500
WA_STALL       = 15

# sensor defaults
# ค่าเริ่มต้นของเซนเซอร์
REFLECT_WHITE  = 85
REFLECT_BLACK  = 10

# Gain การเดินตามเส้น (Tuned for Hub 3 - เซ็นเซอร์ไวขึ้น)
LF_KP_LOW      = 15.0         # ~200 power
LF_KD_LOW      = 8.0
LF_KP_MED      = 25.0         # ~350 power
LF_KD_MED      = 12.0
LF_KP_HIGH     = 35.0         # ~500 power
LF_KD_HIGH     = 18.0
LF_KGYRO       = 4.0

# junction
# การตรวจจับทางแยก
JUNC_OFFSET    = 15
JUNC_CONFIRM   = 5

# landmark snap tolerance
# ความคลาดเคลื่อนที่ยอมรับได้ในการบันทึกตำแหน่งอ้างอิง
LM_TOL_CM      = 4.5


# battery
# แบตเตอรี่
BAT_FULL_MV    = 8300
BAT_WARN_MV    = 7400
BAT_MIN_MV     = 6400
# >>visuals
# ANSI Colors for Console
CLR_RED    = "\033[91m"
CLR_GRN    = "\033[92m"
CLR_YLW    = "\033[93m"
CLR_BLU    = "\033[94m"
CLR_GRY    = "\033[90m"
CLR_RST    = "\033[0m"
