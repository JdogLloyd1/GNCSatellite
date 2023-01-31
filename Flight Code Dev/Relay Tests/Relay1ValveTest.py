# Relay 1 Valve Test

import time

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
# GPIO Pins
# HongWei1 = GPIO4
# HongWei2 = GPIO5 - WOKRS
# HongWei3 = GPIO6 - WORKS
# HongWei4 = GPIO26 - WORKS
# Songle1 = GPIO22 - WORKS
# Songle2 = GPIO23 - WORKS
# Songle3 = GPIO24 - WORKS
# Songle4 = GPIO25 - WORKS

i = 4
# Pin Setup
GPIO.setup(i, GPIO.OUT)

# High voltage closes circuit, low voltage opens circuit 

GPIO.output(i, GPIO.LOW)
print("on")
time.sleep(2)
GPIO.output(i, GPIO.HIGH)
print("off")
time.sleep(2)
GPIO.output(i, GPIO.LOW)
print("on")
time.sleep(2)
GPIO.output(i, GPIO.HIGH)
print("off")
time.sleep(2)
# turn off all relays outside of the loop
GPIO.output(i, GPIO.HIGH)
print("off")


GPIO.cleanup()

