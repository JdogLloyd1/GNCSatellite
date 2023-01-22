# Live script to convert 9 axis IMU data into Euler angles in real time
# Live plots

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
# GPIO.setup()
# SCL = GPIO3
# SDA = GPIO2
# VCC = GPIO1
# GND = GPIO6

sample_rate = 100 # Hertz

timestamp = np.zeros((1,1))
a_shape = (1,3)
gyroscope = np.zeros(a_shape)
accelerometer = np.zeros(a_shape)
print(type(accelerometer))
magnetometer = np.zeros(a_shape)
delta_time = 1 / sample_rate

# TRY np.empty INSTEAD OF ZEROS

# Instantiate algorithms
offset = imufusion.Offset(sample_rate)
ahrs = imufusion.Ahrs()

ahrs.settings = imufusion.Settings(0.5,  # gain
                                   10,  # acceleration rejection
                                   20,  # magnetic rejection
                                   5 * sample_rate)  # rejection timeout = 5 seconds

euler = np.empty((len(timestamp), 3))
internal_states = np.empty((len(timestamp), 6))
flags = np.empty((len(timestamp), 5))

start_time = time.time()
index = 0

while time.time() < start_time + 10:
    
    index += 1
    
    accel_raw = mpu9250.readAccel()
    gyro_raw = mpu9250.readGyro()
    mag_raw = mpu9250.readMagnet()
    
    np.append(timestamp, time.time()-start_time)
    np.append(accelerometer, [[accel_raw['x'], accel_raw['y'], accel_raw['z']]],axis=0)
    x=np.array([[1,2,3]])
    y=np.array([[1,2,3]])
    np.concatenate((y, x), axis=0)
    np.append(gyroscope, [[gyro_raw['x'], gyro_raw['y'], gyro_raw['z']]],axis=0)
    np.append(magnetometer, [[mag_raw['x'], mag_raw['y'], mag_raw['z']]],axis=0)
    print(y)
    print(timestamp)
    print(accelerometer)
    print(accel_raw)
    print(accel_raw['x'])
    print(gyroscope)
    print(magnetometer)
    gyroscope[index] = offset.update(gyroscope[[index]])

    ahrs.update(gyroscope[index], accelerometer[index], magnetometer[index], delta_time)

    euler[index] = ahrs.quaternion.to_euler()

    ahrs_internal_states = ahrs.internal_states
    internal_states[index] = np.array([ahrs_internal_states.acceleration_error,
                                          ahrs_internal_states.accelerometer_ignored,
                                          ahrs_internal_states.acceleration_rejection_timer,
                                          ahrs_internal_states.magnetic_error,
                                          ahrs_internal_states.magnetometer_ignored,
                                          ahrs_internal_states.magnetic_rejection_timer])

    ahrs_flags = ahrs.flags
    flags[index] = np.array([ahrs_flags.initialising,
                                ahrs_flags.acceleration_rejection_warning,
                                ahrs_flags.acceleration_rejection_timeout,
                                ahrs_flags.magnetic_rejection_warning,
                                ahrs_flags.magnetic_rejection_timeout])







