from system import pid_lib as P
from system import field as F

async def run(odo, tm):
    print("--- Running Mission 2 ---")
    
    # Drive backwards
    await P.straight(odo, -20)
