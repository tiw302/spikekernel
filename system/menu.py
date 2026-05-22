"""
    menu.py -- on-hub mission selection system.
    
    uses left/right buttons to cycle missions and 
    displays the mission ID on the 5x5 LED matrix.
"""
import hub as _hub
import runloop
import time

async def select_mission(mission_list: list) -> int:
    """
    returns the index of the selected mission.
    - Right Button: Next Mission
    - Left Button: Confirm Selection
    """
    idx = 0
    n = len(mission_list)
    import config as C
    print(f"{C.CLR_BLU}[menu]{C.CLR_RST} select from {n} missions")
    
    # Initial display
    _hub.light_matrix.write(str(idx + 1))
    
    from hub import button
    
    # Wait for buttons to be released first
    while button.pressed(button.LEFT) or button.pressed(button.RIGHT):
        await runloop.sleep_ms(50)

    while True:
        # Display current mission ID
        _hub.light_matrix.write(str(idx + 1))
        
        # Cycle through missions with Right button
        if button.pressed(button.RIGHT):
            idx = (idx + 1) % n
            print(f"{C.CLR_BLU}[menu]{C.CLR_RST} cycle to mission {idx + 1}")
            # Debounce
            while button.pressed(button.RIGHT): await runloop.sleep_ms(10)
            await runloop.sleep_ms(100)
            
        # Confirm with Left button
        if button.pressed(button.LEFT):
            print(f"{C.CLR_BLU}[menu]{C.CLR_RST} mission {idx + 1} confirmed")
            
            # 1. Show Ready State (Dash icon)
            _hub.light_matrix.write("-")
            
            # Wait for all buttons to be released
            while button.pressed(button.LEFT) or button.pressed(button.RIGHT):
                await runloop.sleep_ms(50)
                
            print(f"{C.CLR_BLU}[menu]{C.CLR_RST} waiting for RIGHT button to start...")
            
            # 2. Wait for Start (Press Right Button)
            while not button.pressed(button.RIGHT):
                await runloop.sleep_ms(50)
                
            print(f"{C.CLR_GRN}[menu]{C.CLR_RST} STARTING MISSION {idx + 1}!")
            
            # Visual feedback (Blink fast)
            for _ in range(3):
                _hub.light_matrix.clear()
                await runloop.sleep_ms(100)
                _hub.light_matrix.write(str(idx + 1))
                await runloop.sleep_ms(100)
            return idx
            
        await runloop.sleep_ms(50)
