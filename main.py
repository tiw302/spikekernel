"""
    main.py -- pro entry point for wro missions.

    how to use:
    -----------
    1. define your mission logic in mission().
    2. use tm.start(coro) for non-blocking attachment movements.
    3. run the script and use left/right buttons for emergency stop.
"""

import runloop
import pid_lib    as P
import sensor_lib as S
import setup      as SETUP
import config     as C

# ███    ███ ██ ███████ ███████ ██  ██████  ███    ██ 
# ████  ████ ██ ██      ██      ██ ██    ██ ████   ██ 
# ██ ████ ██ ██ ███████ ███████ ██ ██    ██ ██ ██  ██ 
# ██  ██  ██ ██      ██      ██ ██ ██    ██ ██  ██ ██ 
# ██      ██ ██ ███████ ███████ ██  ██████  ██   ████ 
#
# >>mission logic

async def mission(odo, tm):
    """
    write your mission here
    use tm.start(coro) for background tasks
    """
    await P.straight(odo, 50)
    await P.turn(odo, 90)
    
    # example background task:
    # tm.start(P.motor_to_angle(C.PORT_C, 90))
    # await P.straight(odo, 20)
       
    print("mission done")

#  ██████  ██████   ██████ ██   ██ ███████ ███████ ████████ ██████   █████  ████████ ███████ 
# ██    ██ ██   ██ ██      ██   ██ ██      ██         ██    ██   ██ ██   ██    ██    ██      
# ██    ██ ██████  ██      ███████ █████   ███████    ██    ██████  ███████    ██    █████   
# ██    ██ ██   ██ ██      ██   ██ ██           ██    ██    ██   ██ ██   ██    ██    ██      
#  ██████  ██   ██  ██████ ██   ██ ███████ ███████    ██    ██   ██ ██   ██    ██    ███████ 
#
# >>orchestration

async def main():
    # initialize professional tools
    tm = P.TaskManager()
    P._TASKS = tm
    
    # start emergency stop listener in background
    tm.start(P.estop_loop(tm))
    
    try:
        # show memory usage and run full setup
        P.mem_report()
        odo = await SETUP.full_setup()
        if odo is None: return
        
        # start the mission
        await mission(odo, tm)
        
    except P.EStopException:
        print("[main] emergency stop triggered")
    except Exception as e:
        print(f"[main] error: {e}")
    finally:
        # clean up resources and save logs
        tm.cancel_all()
        P.LOGGER.flush()
        print("[main] exit cleanup")

# kick off the runloop
runloop.run(main())
