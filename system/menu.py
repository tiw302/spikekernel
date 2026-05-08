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
    
    # Wait for buttons to be released first
    while _hub.button.left.is_pressed() or _hub.button.right.is_pressed():
        await runloop.sleep_ms(50)

    while True:
        # Display current mission ID
        _hub.light_matrix.write(str(idx + 1))
        
        # Cycle through missions with Right button
        if _hub.button.right.is_pressed():
            idx = (idx + 1) % n
            print(f"{C.CLR_BLU}[menu]{C.CLR_RST} cycle to mission {idx + 1}")
            # Debounce
            while _hub.button.right.is_pressed(): await runloop.sleep_ms(10)
            await runloop.sleep_ms(100)
            
        # Confirm with Left button
        if _hub.button.left.is_pressed():
            print(f"{C.CLR_BLU}[menu]{C.CLR_RST} mission {idx + 1} confirmed")
            # Visual feedback (Blink)
            for _ in range(3):
                _hub.light_matrix.off()
                await runloop.sleep_ms(100)
                _hub.light_matrix.write(str(idx + 1))
                await runloop.sleep_ms(100)
            return idx
            
        await runloop.sleep_ms(50)
