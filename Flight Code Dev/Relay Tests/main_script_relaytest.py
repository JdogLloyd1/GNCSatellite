import random
import math
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
# GPIO Pins
# 1 = GPIO4
# 2 = GPIO5
# 3 = GPIO6
# 4 = GPIO22
# 5 = GPIO23
# 6 = GPIO24
# 7 = GPIO25
# 8 = GPIO26

# Pin Setup
GPIO.setup(4, GPIO.OUT)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)

# turn off all relays outside of the loop
GPIO.output(4, GPIO.LOW)
GPIO.output(5, GPIO.LOW)
GPIO.output(6, GPIO.LOW)
GPIO.output(22, GPIO.LOW)
GPIO.output(23, GPIO.LOW)
GPIO.output(24, GPIO.LOW)
GPIO.output(25, GPIO.LOW)
GPIO.output(26, GPIO.LOW)

prev_valves = [0, 0]
valve_dict = {1:4, 2:5, 3:6, 4:22, 5:23, 6:24, 7:25, 8:26}

num = 1 

if num == 1:
    print("\m")
    print("+ roll")
    valves = [2,4]

if num == 2:
    print("\n")
    print("- roll")
    valves = [6,8]

if num == 3:
    print("\n")
    print("+ pitch")
    valves = [2,8]

if num == 4:
    print("\n")
    print("- pitch")
    valves = [4,6]

if num == 5:
    print("\n")
    print("+ yaw")
    valves = [3,7]

if num == 6:
    print("\n")
    print("- yaw")
    valves = [1,5]
 
time.sleep(5)

for valve in valves:
    if valve != 0:
        GPIO.output(valve_dict[valve], GPIO.HIGH)
    print("valve on")
time.sleep(.5)

for valve in valves:
    if valve != 0:
            GPIO.output(valve_dict[valve], GPIO.LOW)
    print("valve off")   
time.sleep(3)

# turn off all relays outside of the loop
GPIO.output(4, GPIO.LOW)
GPIO.output(5, GPIO.LOW)
GPIO.output(6, GPIO.LOW)
GPIO.output(22, GPIO.LOW)
GPIO.output(23, GPIO.LOW)
GPIO.output(24, GPIO.LOW)
GPIO.output(25, GPIO.LOW)
GPIO.output(26, GPIO.LOW)

GPIO.cleanup()
    
