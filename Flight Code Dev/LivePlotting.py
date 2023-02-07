# This script will test live plotting in MatPlotLib as data is fed over time

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import time
import datetime as dt
import RPi.GPIO as GPIO
import FaBo9Axis_MPU9250

style.use('fivethirtyeight')

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys_accel = []
ys_gyro = []

# Initialize communication with TMP102
mpu9250 = FaBo9Axis_MPU9250.MPU9250()
# Pin Setup
GPIO.setmode(GPIO.BCM)
# GPIO.setup()
# SCL = GPIO3
# SDA = GPIO2
# VCC = GPIO1
# GND = GPIO6

# This function is called periodically from FuncAnimation
def animate_accel(i, xs, ys):

	# Read temperature (Celsius) from accelerometer
    accel_raw = mpu9250.readAccel()
    gyro_raw = mpu9250.readGyro()

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(accel_raw['x'])

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('MPU9250 X Acceleration Raw over Time')
    plt.ylabel('')
    
def animate(i, xs, ys_accel, ys_gyro):

	# Read temperature (Celsius) from accelerometer
	accel_raw = mpu9250.readAccel()
	gyro_raw = mpu9250.readGyro()

	# Add x and y to lists
	xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
	ys_accel.append(accel_raw['x'])
	ys_gyro.append(gyro_raw['x'])

	# Limit x and y lists to 20 items
	xs = xs[-20:]
	ys_accel = ys_accel[-20:]
	ys_gyro = ys_gyro[-20:]

	# Draw x and y lists

	plt.subplot(2,1,1)
	ax.clear()
	
	ax.plot(xs, ys_accel)
	plt.xticks(rotation=45, ha='right')
	plt.subplots_adjust(bottom=0.30)
	plt.title('MPU9250 X Acceleration Raw over Time')
	plt.ylabel('')

	plt.subplot(2,1,2)
	ax.clear()
	ax.plot(xs, ys_gyro)
	plt.xticks(rotation=45, ha='right')
	plt.subplots_adjust(bottom=0.30)
	plt.title('MPU9250 X Gyro Raw over Time')
	plt.ylabel('')

    

# Set up plot to call animate() function periodically
plt.figure(1)
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys_accel, ys_gyro), interval=50)
#xs = []
#plt.figure(2)
#ani2 = animation.FuncAnimation(fig, animate_gyro, fargs=(xs, ys_gyro), interval=50)
plt.show()
