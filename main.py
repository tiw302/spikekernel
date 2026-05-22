import sys
# Make sure the system folder can be resolved if needed, though standard packages should work
from system import kernel, setup
from system import pid_lib as P
from system import field as F
from system.menu import select_mission

# Import missions
from missions import mission_1, mission_2

# Main Mission Handler
async def main_mission(odo, tm):
    # 1. Show Menu to select mission
    missions = [mission_1.run, mission_2.run]
    selected_idx = await select_mission(missions)
    
    # 2. Run the selected mission
    await missions[selected_idx](odo, tm)

# Run through the kernel
kernel.run(main_mission)
