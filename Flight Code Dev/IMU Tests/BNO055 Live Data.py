import time 
import board
import busio
import adafruit_bno055 

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.mode = adafruit_bno055.NDOF_MODE

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

#while sensor.calibrated != True:

#	print("Sys/Gyro/Acc/Mag")
#	print(sensor.calibration_status)
#	print(sensor.calibrated)
#	print("")
#time.sleep(1)

# Create figure for plotting
fig1 = plt.figure(1)
ax1 = fig1.add_subplot(3, 1, 1)
ax2 = fig1.add_subplot(3, 1, 2)
ax3 = fig1.add_subplot(3, 1, 3)

fig2 = plt.figure(2)
ax4 = fig2.add_subplot(3, 1, 1)
ax5 = fig2.add_subplot(3, 1, 2)
ax6 = fig2.add_subplot(3, 1, 3)

xs = []
ys_roll = []
ys_pitch = []
ys_yaw = []
ys_wx = []
ys_wy = []
ys_wz = []
	
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
	
	#Have two options for pitch, use whichever works numerically
	inside_fun = 2*(q0*q2 - q1*q3)
	if abs(inside_fun) < 1:
		theta = math.asin(2*(q0*q2 - q1*q3))
	else:
		theta = -math.pi/2 + 2*math.atan2(math.sqrt(1+2*(q0*q2 - q1*q3)), math.sqrt(1-2*q0*q2-q1*q3)) #pitch rotation
	psi = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2**2 + q3**2)) #yaw rotation

	# Add x and y to lists
	xs.append(time.time())
	ys_roll.append(phi*180/math.pi)
	ys_pitch.append(theta*180/math.pi)
	ys_yaw.append(psi*180/math.pi)

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
	#ax1.ylabel('roll')
	leg1 = ax1.legend()

	#plt.subplot(3,1,2)
	ax2.clear()
	ax2.plot(xs, ys_pitch, label = 'pitch')
	#plt.xticks(rotation=45, ha='right')
	#plt.subplots_adjust(bottom=0.50)
	#plt.title('Pitch')
	#ax2.ylabel('pitch')
	leg2 = ax2.legend()
	
	#plt.subplot(3,1,3)
	ax3.clear()
	ax3.plot(xs, ys_yaw, label = 'yaw')
	#plt.xticks(rotation=45, ha='right')
	#plt.subplots_adjust(bottom=0.50)
	#plt.title('Yaw')
	#ax3.ylabel('yaw')
	leg3 = ax3.legend()
	
	print(sensor.calibrated)
	
def animate_omega(i, xs, ys_wx, ys_wy, ys_wz): #function to plot angular velocities in real time

	#Read quaternion from IMU, translate to z-y-x euler angles
	
	position = sensor.quaternion
	ang_vel = sensor.gyro
	

	# Add x and y to lists
	#xs.append(time.time())
	ys_wx.append(ang_vel[0]*180/math.pi)
	ys_wy.append(ang_vel[1]*180/math.pi)
	ys_wz.append(ang_vel[2]*180/math.pi)

	# Limit x and y lists to 20 items
	xs = xs[-20:]
	ys_wx = ys_wx[-20:]
	ys_wy = ys_wy[-20:]
	ys_wz = ys_wz[-20:]

	# Draw x and y lists

	#plt.subplot(3,1,1)
	ax4.clear()
	ax4.plot(xs, ys_wx, label = 'omega x')
	#plt.xticks(rotation=45, ha='right')
	#plt.subplots_adjust(bottom=0.30)
	#plt.title('Roll')
	#ax1.ylabel('roll')
	leg4 = ax4.legend()

	#plt.subplot(3,1,2)
	ax5.clear()
	ax5.plot(xs, ys_wy, label = 'omega y')
	#plt.xticks(rotation=45, ha='right')
	#plt.subplots_adjust(bottom=0.50)
	#plt.title('Pitch')
	#ax2.ylabel('pitch')
	leg5 = ax5.legend()
	
	#plt.subplot(3,1,3)
	ax6.clear()
	ax6.plot(xs, ys_wz, label = 'omega z')
	#plt.xticks(rotation=45, ha='right')
	#plt.subplots_adjust(bottom=0.50)
	#plt.title('Yaw')
	#ax3.ylabel('yaw')
	leg6 = ax6.legend()
	
	#print(sensor.calibrated)	
	
# Set up plot to call animate() function periodically
ani1 = animation.FuncAnimation(fig1, animate_euler, fargs=(xs, ys_roll, ys_pitch, ys_yaw), interval=1000)
ani2 = animation.FuncAnimation(fig2, animate_omega, fargs=(xs, ys_wx, ys_wy, ys_wz), interval=1000)
plt.show()
