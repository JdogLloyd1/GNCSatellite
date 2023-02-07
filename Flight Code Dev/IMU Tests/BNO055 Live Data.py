import time 
import board
import busio
import adafruit_bno055 

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF
print("Quaternion: {}".format(sensor.quaternion))


import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
#import time
import datetime as dt



start_time = time.time()

style.use('fivethirtyeight')

# Create figure for plotting
fig = plt.figure()
ax1 = fig.add_subplot(3, 1, 1)
ax2 = fig.add_subplot(3, 1, 2)
ax3 = fig.add_subplot(3, 1, 3)
xs = []
ys_roll = []
ys_pitch = []
ys_yaw = []
	
print("Quaternion: {}".format(sensor.quaternion))

def animate_euler(i, xs, ys_roll, ys_pitch, ys_yaw): #function to plot euler angles in real time

	#Read quaternion from IMU, translate to z-y-x euler angles
	
	position = sensor.quaternion
	ang_vel = sensor.gyro
	
	q0 = position[0]
	q1 = position[1]
	q2 = position[2]
	q3 = position[3]
	
	phi = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1**2 + q2**2)) #roll rotation
	theta = -math.pi/2 + 2*math.atan2(math.sqrt(1+2*(q0*q2 - q1*q3)), math.sqrt(1-2*q0*q2-q1*q3)) #pitch rotation
	psi = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2**2 + q3**2)) #yaw rotation

	# Add x and y to lists
	xs.append(time.time())
	ys_roll.append(phi)
	ys_pitch.append(theta)
	ys_yaw.append(psi)

	# Limit x and y lists to 20 items
	xs = xs[-20:]
	ys_roll = ys_roll[-20:]
	ys_pitch = ys_pitch[-20:]
	ys_yaw = ys_yaw[-20:]

	# Draw x and y lists

	#plt.subplot(3,1,1)
	ax1.clear()
	ax1.plot(xs, ys_roll, label = 'roll')
	#plt.xticks(rotation=45, ha='right')
	#plt.subplots_adjust(bottom=0.30)
	#plt.title('Roll')
	plt.ylabel('roll')
	leg = plt.legend()

	#plt.subplot(3,1,2)
	ax2.clear()
	ax2.plot(xs, ys_pitch, label = 'pitch')
	#plt.xticks(rotation=45, ha='right')
	#plt.subplots_adjust(bottom=0.50)
	#plt.title('Pitch')
	plt.ylabel('pitch')
	leg = plt.legend()
	
	#plt.subplot(3,1,3)
	ax3.clear()
	ax3.plot(xs, ys_yaw, label = 'yaw')
	#plt.xticks(rotation=45, ha='right')
	#plt.subplots_adjust(bottom=0.50)
	#plt.title('Yaw')
	plt.ylabel('')
	leg = plt.legend()
	
	
	
# Set up plot to call animate() function periodically
plt.figure(1)
ani = animation.FuncAnimation(fig, animate_euler, fargs=(xs, ys_roll, ys_pitch, ys_yaw), interval=1000)
plt.show()
