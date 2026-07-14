#!/usr/bin/env pybricks-micropython
# Spike prime / inventor hub — Pybricks 4.0 (https://beta.pybricks.com/)
# My github account: https://github.com/tiw302, My ig account: @tiw3025k_

#                    (GRIPPER)        [TOP VIEW]
#                    [Port A]
#                  .----------.
#                  | [B]  [C] |  <- sensors (L/R)
#         .--------.----------.--------.
#         |  [D]   |  SPIKE   |  [E]   |
#         | (Left  |   HUB    | (Right |
#         | Motor) |   [F]    | Motor) |
#         |        |(MAIN ARM)|        |
#         '--------'----------'--------'

"""
* debug module & menu system:                                        [ENGLISH]
    used for testing sensors and checking hardware before a match.
    features a spike button menu to select the operating mode without restarting.
    
    [left button]   = debug sensor (read reflection continuously)
    [center button] = calibrate sensors (run calibration sequence)
    [right button]  = system & battery check
    [bluetooth btn] = exit current mode or exit debug loop

.___________________________________________________________________________________________.

* ระบบตรวจสอบและตั้งค่าก่อนแข่ง:                                           [THAI]
    ใช้สำหรับเทสเซ็นเซอร์และเช็คระบบฮาร์ดแวร์ก่อนลงสนามจริง
    มีระบบเมนูให้กดปุ่มบน SPIKE เพื่อเลือกโหมดทำงานได้ทันที
    
    [ปุ่มซ้าย]   = อ่านค่าแสงจากเซ็นเซอร์ (debug sensors)
    [ปุ่มกลาง]  = เข้าโหมดคาลิเบรต (calibrate sensors)
    [ปุ่มขวา]   = เช็คระบบและแบตเตอรี่ (system check)
    [ปุ่มบลูทูธ] = ออกจากโหมดปัจจุบัน หรือออกจากเมนู
"""

import gc
from pybricks.parameters import Button, Color
from pybricks.tools import wait, StopWatch
from main import Robot

#  ____  _____ ____  _   _  ____ 
# |  _ \| ____| __ )| | | |/ ___|
# | | | |  _| |  _ \| | | | |  _ 
# | |_| | |___| |_) | |_| | |_| |
# |____/|_____|____/ \___/ \____|
#
# >> debug and menu execution
if __name__ == "__main__":
    robot = Robot()
    hub = robot.hub
    hub.speaker.beep(1047, 200)
    
    def draw_menu():
        hub.display.char("?")
        print("\n" + "="*40)
        print("      SPIKE SENSOR DEBUG MODE")
        print("="*40)
        print("[LEFT]      debug sensor reading")
        print("[CENTER]    calibrate sensors")
        print("[RIGHT]     system & battery check")
        print("[BLUETOOTH] exit debug menu / mode")
        print("waiting for button input...")
        print("="*40 + "\n")

    draw_menu()
    
    while True:
        pressed = hub.buttons.pressed()
        
        #  ██████  ███████ ███    ██ ███████  ██████  ██████  ███████ 
        # ██       ██      ████   ██ ██      ██    ██ ██   ██ ██      
        #  █████   █████   ██ ██  ██ ███████ ██    ██ ██████  ███████ 
        #      ██  ██      ██  ██ ██      ██ ██    ██ ██   ██      ██ 
        # ██████   ███████ ██   ████ ███████  ██████  ██   ██ ███████ 
        
        #  ___ ___ _  _ ___  ___  ___   _____ ___ ___ _____
        # / __| __| \| / __/  _ \| _ \ |_   _| __/ __|_   _|
        # \__ \ _|| .` \__ \ (_) |   /   | | | _|\__ \ | |
        # |___/___|_|\_|___/\___/|_|_\   |_| |___|___/ |_|
        #
        # >> mode 1: debug sensors (left button)
        if Button.LEFT in pressed:
            hub.speaker.beep(1319, 200)
            hub.display.char("S")
            print("[ROBOT] mode: debug sensor (press BLUETOOTH to exit)")
            
            # debounce: wait for left to be released
            while Button.LEFT in hub.buttons.pressed():
                wait(10)
            
            sl_ref = robot.sensor_l.reflection
            sr_ref = robot.sensor_r.reflection
            
            while True:
                # check exit button FIRST before any wait
                if Button.BLUETOOTH in hub.buttons.pressed():
                    print("[ROBOT] exiting debug mode.")
                    break
                
                # zero-allocation sensor reading
                vl = sl_ref()
                vr = sr_ref()
                print(f"[ROBOT] S_L(Port.B) : {vl:3} | S_R(Port.C) : {vr:3}")
                
                # visual feedback on the hub's 5x5 led matrix
                # 0-100 mapped to brightness of the left and right center pixels
                hub.display.pixel(1, 2, vl) # left sensor
                hub.display.pixel(3, 2, vr) # right sensor
                
                wait(100)
            
            hub.speaker.beep(800, 200)
            # redraw menu after mode finishes
            draw_menu()
            # debounce: wait for bt to be released AFTER beep
            while Button.BLUETOOTH in hub.buttons.pressed():
                wait(10)
        
        #   ___   _   _    ___ ___ ___    _ _____ ___
        #  / __| /_\ | |  |_ _| _ ) _ \  /_\_   _| __|
        # | (__ / _ \| |__ | || _ \   / / _ \| | | _|
        #  \___/_/ \_\____|___|___/_|_\/_/ \_\_| |___|
        #
        # >> mode 2: calibrate sensors (center button)
        elif Button.CENTER in pressed:
            hub.speaker.beep(1319, 200)
            hub.display.char("C")
            
            print("[ROBOT] Start: Calibrating (4s)...")
            print("[ROBOT] Calibrating: place on WHITE and press CENTER...")
            # debounce: wait for center to be released from menu
            while Button.CENTER in hub.buttons.pressed(): wait(10)
            # wait for user to press center to confirm white
            while Button.CENTER not in hub.buttons.pressed(): wait(10)
            
            hub.speaker.beep(700, 100)
            wait(200)

            w_sum = 0; w_cnt = 0; sw = StopWatch()
            sl_ref = robot.sensor_l.reflection
            sr_ref = robot.sensor_r.reflection
            
            while sw.time() < 2000:
                w_sum += (sl_ref() + sr_ref()) / 2
                w_cnt += 1; wait(20)
            w_res = int(w_sum // w_cnt)
            print(f"[ROBOT] White = {w_res}")

            print("[ROBOT] Calibrating: place on BLACK and press CENTER...")
            while Button.CENTER in hub.buttons.pressed(): wait(10)
            while Button.CENTER not in hub.buttons.pressed(): wait(10)
            
            hub.speaker.beep(400, 100)
            wait(200)

            b_sum = 0; b_cnt = 0; sw.reset()
            while sw.time() < 2000:
                b_sum += (sl_ref() + sr_ref()) / 2
                b_cnt += 1; wait(20)
            b_res = int(b_sum // b_cnt)
            print(f"[ROBOT] Black = {b_res}")
            
            mid = (w_res + b_res) // 2
            
            robot.hub.speaker.beep(800, 200)
            print("[ROBOT] Done: Calibrating")
            
            # print reminder to console
            print("\n" + "="*40)
            print(f">>> WHITE_LIGHT = {w_res}")
            print(f">>> BLACK_LIGHT = {b_res}")
            print(f">>> LINE_EDGE   = {mid}")
            print("please update main.py with the values above.")
            print("="*40 + "\n")
            
            wait(2000)
            # redraw menu after mode finishes
            draw_menu()
            while Button.CENTER in hub.buttons.pressed(): wait(10)
        
        #  ███████ ██    ██ ███████ ████████ ███████ ███    ███ 
        # ██        ██  ██  ██         ██    ██      ████  ████ 
        # ███████    ████   ███████    ██    █████   ██ ████ ██ 
        #      ██     ██         ██    ██    ██      ██  ██  ██ 
        # ███████     ██    ███████    ██    ███████ ██      ██ 
        
        #  ___ _   _ ___ _____ ___ __  __   ___ _  _ ___ ___
        # / __| | | / __|_   _| __|  \/  | |_ _| \| | __/ _ \
        # \__ \ |_| \__ \ | | | _|| |\/| |  | || .` | _| (_) |
        # |___/\__, |___/ |_| |___|_|  |_| |___|_|\_|_| \___/
        #      |___/
        #
        # >> mode 3: system check (right button)
        elif Button.RIGHT in pressed:
            hub.speaker.beep(1047, 200)
            hub.display.char("B")
            # debounce: wait for right to be released
            while Button.RIGHT in hub.buttons.pressed():
                wait(10)
                
            print("[ROBOT] mode: system check (press BLUETOOTH to exit)")
            while True:
                # check exit button FIRST before reading sensors
                if Button.BLUETOOTH in hub.buttons.pressed():
                    hub.speaker.beep(800, 200)
                    break
                
                volts = hub.battery.voltage() / 1000.0
                amps = hub.battery.current() / 1000.0
                percent = (volts - 7.0) / (8.2 - 7.0) * 100
                if percent < 0: percent = 0
                if percent > 100: percent = 100
                
                # get raw ram allocation via garbage collector
                free_kb = gc.mem_free() / 1024.0
                alloc_kb = gc.mem_alloc() / 1024.0
                
                # print to console so user can see it on computer screen too
                print(f"[ROBOT] BAT: {volts:.2f}V {percent:.0f}% | CUR: {amps:.2f}A | RAM: {free_kb:.0f}KB Free")
                
                wait(500)
                
            # redraw menu after mode finishes
            draw_menu()
            # debounce: wait for bt to be released AFTER beep
            while Button.BLUETOOTH in hub.buttons.pressed():
                wait(10)
        
        # ███████ ██   ██ ██ ████████ 
        # ██       ██ ██  ██    ██    
        # █████     ███   ██    ██    
        # ██       ██ ██  ██    ██    
        # ███████ ██   ██ ██    ██    
        
        # >> mode 4: exit (bluetooth button in main menu)
        elif Button.BLUETOOTH in pressed:
            hub.speaker.beep(800, 200)
            hub.display.off()
            print("[ROBOT] exiting debug menu.")
            wait(500)
            break
        
        wait(10)
