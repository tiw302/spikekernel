import sys
# make sure the system folder can be resolved if needed, though standard packages should work
from system import kernel, setup
from system import pid_lib as P
from system import field as F
from system.menu import select_mission

# import missions
from missions import mission_1, mission_2

# main Mission Handler
async def main_mission(odo, tm):
    # 1. show menu to select mission
    missions = [mission_1.run, mission_2.run]
    selected_idx = await select_mission(missions)
    
    # 2. run the selected mission
    await missions[selected_idx](odo, tm)

# run through the kernel
kernel.run(main_mission)
