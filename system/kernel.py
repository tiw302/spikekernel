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
        import config as C
        print(f"{C.CLR_RED}[kernel]{C.CLR_RST} emergency stop triggered")
    except Exception as e:
        import config as C
        print(f"{C.CLR_RED}[kernel]{C.CLR_RST} error: {e}")
    finally:
        tm.cancel_all()
        import config as C
        print(f"{C.CLR_YLW}[kernel]{C.CLR_RST} exit cleanup")

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
    import gc
    gc.collect()
    print(f"[kernel] starting... RAM free: {gc.mem_free()} bytes")
    
    try:
        runloop.run(_entry(mission_func))
    except Exception as e:
        print(f"[kernel] crash: {e}")
    finally:
        gc.collect()
        print(f"[kernel] shutdown. RAM free: {gc.mem_free()} bytes")
