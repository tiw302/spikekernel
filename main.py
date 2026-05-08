import kernel, setup
import pid_lib as P
import field as F
from menu import select_mission

async def mission_1(odo, tm):
    print("--- Running Mission 1 ---")
    
    # Use coordinates from field.py
    await P.straight(odo, 50)
    
    await P.turn(odo, F.TARGET_OBSTACLE_1[2])
    await P.goto_xy(odo, F.TARGET_OBSTACLE_1[0], F.TARGET_OBSTACLE_1[1])
    

async def mission_2(odo, tm):
    print("--- Running Mission 2 ---")
    await P.straight(odo, -20)

# Main Mission Handler
async def main_mission(odo, tm):
    # 1. Show Menu to select mission
    missions = [mission_1, mission_2]
    selected_idx = await select_mission(missions)
    
    # 2. Run the selected mission
    await missions[selected_idx](odo, tm)

# Run through the kernel
kernel.run(main_mission)
