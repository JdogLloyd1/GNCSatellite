# Numpy test script

import time
import sys
import RPi.GPIO as GPIO
import FaBo9Axis_MPU9250
import imufusion
import matplotlib.pyplot as pyplot
import numpy as np

mpu9250 = FaBo9Axis_MPU9250.MPU9250()

# Pin Setup
GPIO.setmode(GPIO.BCM)

accel_raw = mpu9250.readAccel()
gyro_raw = mpu9250.readGyro()
mag_raw = mpu9250.readMagnet()

sample_rate = 100 # Hertz

timestamp = np.zeros((1,1))
a_shape = (1,3)

gyroscope = np.zeros(a_shape)
np.append(gyroscope, [[1, 2, 3]])
print(gyroscope)

list = [[1,2, 3], [4, 5, 6]]
#arr = np.array(list)
#print(arr)

arr = np.empty([1,3])
arr = np.append(arr, [[0,2,4],[6,8,10]], axis = 0)
arr = np.append(arr, [[5,7,9],[13,15,17]], axis = 0)
arr = np.append(arr, [[accel_raw['x'], accel_raw['y'], accel_raw['z']]], axis = 0)
print(arr)
