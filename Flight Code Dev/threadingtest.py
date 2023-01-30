import time
import sys
import RPi.GPIO as GPIO
import imufusion
import matplotlib.pyplot as pyplot
import numpy as np
import csv
from threading import Thread

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
            
def 

def readIMU():
    
    return ? 
            
timeEnd = time.time() + 10 # seconds
start_time = time.time()

#initialize numpy arrays
#timestamp = np.empty([1,1])
#gyroscope = np.empty([1,3])
#accelerometer = np.empty([1,3])
#magnetometer = np.empty([1,3])
data_lst = []

while time.time() < timeEnd:
	#print(time.time()-start_time)
	accel_raw = mpu9250.readAccel()
	gyro_raw = mpu9250.readGyro()
	mag_raw = mpu9250.readMagnet()

	data_vec = [time.time()-start_time, accel_raw['x'], accel_raw['y'], accel_raw['z'], gyro_raw['x'], \
	gyro_raw['y'], gyro_raw['z'], mag_raw['x'], mag_raw['y'], mag_raw['z']]
	now = time.time()
	delta_t = time.time()-start_time
	data_lst.append(data_vec) 
	elapsed_time = time.time() - now
	time.sleep(.01-elapsed_time) #want to run at 100 Hz


write_csv('IMU sample data.csv',data_lst)
