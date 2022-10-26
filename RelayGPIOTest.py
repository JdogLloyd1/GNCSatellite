import time

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
# GPIO Pins
# HongWei1 = GPIO4
# HongWei2 = GPIO5
# HongWei3 = GPIO6
# HongWei4 = GPIO26
# Songle1 = GPIO22
# Songle2 = GPIO23
# Songle3 = GPIO24
# Songle4 = GPIO25

# Pin Setup
GPIO.setup(4, GPIO.OUT)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)

# High voltage closes circuit, low voltage opens circuit 
for i in range(10):
    GPIO.output(4, GPIO.LOW)
    GPIO.output(5, GPIO.LOW)
    GPIO.output(6, GPIO.LOW)
    GPIO.output(26, GPIO.LOW)
    GPIO.output(22, GPIO.LOW)
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(25, GPIO.LOW)
    time.sleep(.1)
    GPIO.output(4, GPIO.HIGH)
    GPIO.output(5, GPIO.HIGH)
    GPIO.output(6, GPIO.HIGH)
    GPIO.output(26, GPIO.HIGH)
    GPIO.output(22, GPIO.HIGH)
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.HIGH)
    GPIO.output(25, GPIO.HIGH)
    time.sleep(.1)

# turn off all relays outside of the loop
GPIO.output(4, GPIO.LOW)
GPIO.output(5, GPIO.LOW)
GPIO.output(6, GPIO.LOW)
GPIO.output(26, GPIO.LOW)
GPIO.output(22, GPIO.LOW)
GPIO.output(23, GPIO.LOW)
GPIO.output(24, GPIO.LOW)
GPIO.output(25, GPIO.LOW)

GPIO.cleanup()
