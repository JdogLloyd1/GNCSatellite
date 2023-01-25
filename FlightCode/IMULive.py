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

timestamp = np.empty([1,1])
gyroscope = np.empty([1,3])
accelerometer = np.empty([1,3])
magnetometer = np.empty([1,3])
delta_time = 1 / sample_rate

# TRY np.empty INSTEAD OF ZEROS

# Instantiate algorithms
offset = imufusion.Offset(sample_rate)
ahrs = imufusion.Ahrs()

ahrs.settings = imufusion.Settings(0.5,  # gain
                                   10,  # acceleration rejection
                                   20,  # magnetic rejection
                                   5 * sample_rate)  # rejection timeout = 5 seconds
end_time = 1

euler = np.empty((1, 3))
internal_states = np.empty((1, 6))
flags = np.empty((1, 5))

start_time = time.time()
index = 1

while time.time() < start_time + end_time:
    
    
    accel_raw = mpu9250.readAccel()
    gyro_raw = mpu9250.readGyro()
    mag_raw = mpu9250.readMagnet()
    
    timestamp = np.append(timestamp, [[time.time()-start_time]], axis = 0)
    accelerometer = np.append(accelerometer, [[accel_raw['x'], accel_raw['y'], accel_raw['z']]], axis = 0)
    gyroscope = np.append(gyroscope, [[gyro_raw['x'], gyro_raw['y'], gyro_raw['z']]], axis = 0)
    magnetometer = np.append(magnetometer, [[mag_raw['x'], mag_raw['y'], mag_raw['z']]], axis = 0)
    
    
    gyroscope[index] = offset.update(gyroscope[index])

    ahrs.update(gyroscope[index], accelerometer[index], magnetometer[index], delta_time)

    euler = np.append(euler, [ahrs.quaternion.to_euler()], axis = 0)

    ahrs_internal_states = ahrs.internal_states
    internal_states = np.append(internal_states, [[ahrs_internal_states.acceleration_error,
                                          ahrs_internal_states.accelerometer_ignored,
                                          ahrs_internal_states.acceleration_rejection_timer,
                                          ahrs_internal_states.magnetic_error,
                                          ahrs_internal_states.magnetometer_ignored,
                                          ahrs_internal_states.magnetic_rejection_timer]], axis = 0)

    ahrs_flags = ahrs.flags
    flags = np.append(flags, [[ahrs_flags.initialising,
                                ahrs_flags.acceleration_rejection_warning,
                                ahrs_flags.acceleration_rejection_timeout,
                                ahrs_flags.magnetic_rejection_warning,
                                ahrs_flags.magnetic_rejection_timeout]], axis = 0)

    index += 1

#print(euler)

# Plot Euler angles
#figure, axes = pyplot.plot()

#figure.suptitle("Euler angles, internal states, and flags")

pyplot.plot(timestamp, euler[:, 0], "tab:red", label="Roll")
pyplot.plot(timestamp, euler[:, 1], "tab:green", label="Pitch")
pyplot.plot(timestamp, euler[:, 2], "tab:blue", label="Yaw")
pyplot.legend()

pyplot.show()


