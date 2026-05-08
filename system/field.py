"""
    field.py -- competition map definitions.
    
    use this file to store all X, Y coordinates and headings 
    for your mission targets. separate the field logic 
    from the movement logic.
"""

# STARTING POSITIONS
START_HOME_A = (0, 0, 0)
START_HOME_B = (0, 150, 90)

# MISSION TARGETS (Example: X, Y, Heading)
TARGET_OBSTACLE_1 = (120.5, 45.0, 0)
TARGET_DROP_ZONE  = (200.0, 100.0, -90)

# LANDMARKS (For position snapping)
LM_GREEN_LINE_1 = 120.0  # X coordinate of the first green line
LM_BLUE_AREA_Y  = 80.0   # Y coordinate of the blue zone edge

# ATTACHMENT POSITIONS
ARM_UP    = 0
ARM_DOWN  = -450
CLAW_OPEN = 100
CLAW_GRIP = -200
