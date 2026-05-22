from system import pid_lib as P
from system import field as F

async def run(odo, tm):
    print("--- Running Mission 1 ---")
    
    # Use coordinates from field.py
    await P.straight(odo, 50)
    
    await P.turn(odo, F.TARGET_OBSTACLE_1[2])
    await P.goto_xy(odo, F.TARGET_OBSTACLE_1[0], F.TARGET_OBSTACLE_1[1])
