"""
    setup.py -- pre-mission initialization sequence.

    technical background:
    ---------------------
    this module handles the robot's "birth" sequence each time 
    the program starts. it ensures the battery is healthy, 
    calibrates the gyro, and prepares the odometry system.

    workflow:
    ---------
    1. battery check (gatekeeper)
    2. gyro calibration (stationary)
    3. sensor calibration (white/black normalization)
    4. competition check (hardware validation)
    5. wall alignment (origin reset)
"""

import pid_lib    as P
import sensor_lib as S
import runloop, time
import hub as _hub
from hub import motion_sensor
import motor_pair
import config as C

# ██████   █████  ████████ ████████ ███████ ██████  ██    ██ 
# ██   ██ ██   ██    ██       ██    ██      ██   ██  ██  ██  
# ██████  ███████    ██       ██    █████   ██████    ████   
# ██   ██ ██   ██    ██       ██    ██      ██   ██    ██    
# ██████  ██   ██    ██       ██    ███████ ██   ██    ██    
#
# >>battery

async def check_battery(warn_only:bool=False)->bool:
    v=_hub.battery.voltage()
    pct=(v-C.BAT_MIN_MV)/(C.BAT_FULL_MV-C.BAT_MIN_MV)*100
    ok = v > C.BAT_MIN_MV
    col = C.CLR_GRN if ok else C.CLR_RED
    print(f"{col}[bat]{C.CLR_RST} {v}mv  {pct:.0f}%  {'ok' if ok else 'LOW'}")
    if not ok:
        _hub.sound.beep(200, 500) # Low pitch warning
    else:
        _hub.sound.beep(600, 100) # High pitch ok
    return ok or warn_only

#  ██████  ██    ██ ██████   ██████  
# ██       ██    ██ ██   ██ ██    ██ 
# ██   ███  ██  ██  ██████  ██    ██ 
# ██    ██    ██    ██   ██ ██    ██ 
#  ██████     ██    ██   ██  ██████  
#
# >>gyro

async def gyro_init(preheat_ms:int=5000, samples:int=300)->float:
    print("[setup] gyro init -- DO NOT MOVE")
    _hub.light_matrix.show_image(_hub.light_matrix.IMAGE_SQUARE) # Show warning icon
    bias=await P.calibrate_gyro(preheat_ms=preheat_ms,samples=samples)
    _hub.sound.beep(880, 200) # Success beep
    _hub.light_matrix.off()
    return bias

# ███████ ███████ ███    ██ ███████  ██████  ██████  
# ██      ██      ████   ██ ██      ██    ██ ██   ██ 
# ███████ █████   ██ ██  ██ ███████ ██    ██ ██████  
#      ██ ██      ██  ██ ██      ██ ██    ██ ██   ██ 
# ███████ ███████ ██   ████ ███████  ██████  ██   ██ 
#
# >>sensors

async def sensor_setup(auto:bool=False)->None:
    """
    auto=False -- manual: place on white 3s, then black 3s
    auto=True  -- sweep: drive slowly over a line
    """
    if auto:
        await S.CAL.sweep(C.PORT_C1); await S.CAL.sweep(C.PORT_C2)
    else:
        print(f"{C.CLR_BLU}[setup]{C.CLR_RST} place both sensors on WHITE (3s)..."); time.sleep_ms(3000)
        S.CAL.white(C.PORT_C1,n=30); S.CAL.white(C.PORT_C2,n=30)
        print(f"{C.CLR_BLU}[setup]{C.CLR_RST} place both sensors on BLACK (3s)..."); time.sleep_ms(3000)
        S.CAL.black(C.PORT_C1,n=30); S.CAL.black(C.PORT_C2,n=30)
    S.sensor_report()

#  ██████  ██████   ██████  
# ██    ██ ██   ██ ██    ██ 
# ██    ██ ██   ██ ██    ██ 
# ██    ██ ██   ██ ██    ██ 
#  ██████  ██████   ██████  
#
# >>odometry

async def init_odometry(x:float=0.0, y:float=0.0, h:float=0.0)->P.Odometry:
    odo=P.Odometry(); odo.setup(); odo.reset(x=x,y=y,h=h)
    return odo

async def align_and_reset(odo:P.Odometry,
                           x:float=0.0, y:float=0.0, h:float=0.0)->None:
    await P.wall_align()
    odo.reset(x=x,y=y,h=h)

# ███████ ███████ ████████ ██    ██ ██████  
# ██      ██         ██    ██    ██ ██   ██ 
# ███████ █████      ██    ██    ██ ██████  
#      ██ ██         ██    ██    ██ ██      
# ███████ ███████    ██     ██████  ██      
#
# >>full setup

async def full_setup(skip_sensor_cal:bool=False,
                     skip_comp_check:bool=False,
                     x:float=0.0, y:float=0.0, h:float=0.0) -> P.Odometry | None:
    """
    full pre-mission sequence:
      1. battery check
      2. gyro init
      3. sensor calibration
      4. competition check (straight + turn test)
      5. wall align + reset origin
    returns odometry if all ok, none if aborted
    """
    print(f"\n{C.CLR_BLU}[setup]{C.CLR_RST} === full setup ===")

    if not await check_battery():
        print(f"{C.CLR_RED}[setup]{C.CLR_RST} battery too low -- charge first"); return None

    await gyro_init()

    if not skip_sensor_cal:
        await sensor_setup()

    if not skip_comp_check:
        if not await P.comp_check():
            return None

    odo=await init_odometry(x,y,h)
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    await align_and_reset(odo,x,y,h)
    print(f"{C.CLR_GRN}=== setup done -- ready ==={C.CLR_RST}\n")
    _hub.sound.beep(1000, 100)
    await runloop.sleep_ms(100)
    _hub.sound.beep(1200, 200) # Celebration melody
    return odo

async def quick_setup(x:float=0.0, y:float=0.0, h:float=0.0) -> P.Odometry:
    """
    fast setup for practice runs -- skips sensor cal and comp check
    gyro preheat only 2s
    """
    print("\n=== quick setup ===")
    await P.calibrate_gyro(preheat_ms=2000, samples=150)
    odo=await init_odometry(x,y,h)
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    await align_and_reset(odo,x,y,h)
    print("=== quick setup done ===\n")
    return odo
