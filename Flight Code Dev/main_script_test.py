import random
import math
import time
from valve_test_script import *
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

prev_valves = [0, 0]
valve_dict = {1:4, 2:5, 3:6, 4:22, 5:23, 6:24, 7:25, 8:26}

for index in range(10):
    num = math.ceil(8*random.random())
    num2 = math.ceil(8*random.random()) 
    new_valves = [num, num2]
    
    if new_valves != prev_valves: #only turn on/off valves if it's different from the last timestep\
        
        for valve in prev_valves:
            if valve != 0:
                GPIO.output(valve_dict[valve], GPIO.LOW)
            
        for valve in new_valves:
            GPIO.output(valve_dict[valve], GPIO.HIGH)
            
    prev_valves = [new_valves[0], new_valves[1]] #reset the loop
        
    print(new_valves)
    time.sleep(1)

# turn off all relays outside of the loop
GPIO.output(4, GPIO.LOW)
#GPIO.output(5, GPIO.LOW)
#GPIO.output(6, GPIO.LOW)
#GPIO.output(22, GPIO.LOW)
GPIO.output(23, GPIO.LOW)
GPIO.output(24, GPIO.LOW)
GPIO.output(25, GPIO.LOW)
GPIO.output(26, GPIO.LOW)

GPIO.cleanup()
    
