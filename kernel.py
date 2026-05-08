"""
    kernel.py -- robot system kernel & orchestration.
"""

import runloop
import pid_lib as P
import setup as SETUP

# ██   ██ ███████ ██████  ███    ██ ███████ ██      
# ██  ██  ██      ██   ██ ████   ██ ██      ██      
# █████   █████   ██████  ██ ██  ██ █████   ██      
# ██  ██  ██      ██   ██ ██  ██ ██ ██      ██      
# ██   ██ ███████ ██   ██ ██   ████ ███████ ███████ 
#
# >>system core

async def _entry(mission_func):
    """internal orchestration logic"""
    tm = P.TaskManager()
    P._TASKS = tm
    tm.start(P.estop_loop(tm))
    
    try:
        P.mem_report()
        odo = await SETUP.full_setup()
        if odo is None: return
        await mission_func(odo, tm)
         
    except P.EStopException:
        print("[kernel] emergency stop triggered")
    except Exception as e:
        print(f"[kernel] error: {e}")
    finally:
        tm.cancel_all()
        P.LOGGER.flush()
        print("[kernel] exit cleanup")

# ██████  ██    ██ ███    ██ ███    ██ ███████ ██████  
# ██   ██ ██    ██ ████   ██ ████   ██ ██      ██   ██ 
# ██████  ██    ██ ██ ██  ██ ██ ██  ██ █████   ██████  
# ██   ██ ██    ██ ██  ██ ██ ██  ██ ██ ██      ██   ██ 
# ██   ██  ██████  ██   ████ ██   ████ ███████ ██   ██ 
#
# >>mission runner

def run(mission_func):
    """
    main entry point to execute a mission
    usage: kernel.run(mission)
    """
    runloop.run(_entry(mission_func))
