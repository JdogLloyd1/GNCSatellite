import time
import sys
import RPi.GPIO as GPIO
# import imufusion
import matplotlib.pyplot as pyplot
import numpy as np
# import csv
from threading import Thread
# Import continuous Threading? 
# Import IMU library

# Pin Setup
GPIO.setmode(GPIO.BCM)
# GPIO.setup()
# SCL = GPIO3
# SDA = GPIO2
# VCC = GPIO1
# GND = GPIO6

def write_csv(filename,data):
    with open(filename,'w',newline='') as csvfile:
        f = csv.writer(csvfile)
        for row in data:
            f.writerow(row)

def readIMU():
	
    # start a while True loop
    
    # Read quaternions
    
    # Process quaternions into Euler Angles
    
    # Append timestamp, angles to lists
    
    # Return lists? 
    
    # Use signal.pause() to wait for next data packet to come through? 

    return ? 
            
THREAD_IMU = Thread(target=readIMU())
THREAD_IMU.start()
# Potentially use Thread.Timer() to start reading IMU after initalization/calibration

#initialize lists
timestamp = []
EULER_Z = []
EULER_Y = []
EULER_X = []

timeEnd = time.time() + 10 # seconds
start_time = time.time()

while time.time() < timeEnd:


    # 

	# want to run at 100 Hz? Maybe slower based on data reading?


 # write_csv('IMU sample data.csv',data_lst)
