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


	
print("Quaternion: {}".format(sensor.quaternion))

def matrix_mult(matrix1, matrix2):

    m = len(matrix1) #number of rows in matrix 1
    K = len(matrix2[0])
    res = [[0 for x in range(K)] for y in range(m)]
     
    # explicit for loops
    for i in range(len(matrix1)):
        for j in range(len(matrix2[0])):
            for k in range(len(matrix2)):
     
                # resulted matrix
                res[i][j] += matrix1[i][k] * matrix2[k][j]
     
    return res
    


	#Read quaternion from IMU, translate to z-y-x euler angles

while time.time() - start_time < 10:	
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
    	theta = -math.pi/2 + 2*math.atan2(math.sqrt(1+2*(q0*q2 - q1*q3)), math.sqrt(1-2*(q0*q2-q1*q3))) #pitch rotation
    psi = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2**2 + q3**2)) #yaw rotation
    
    print("roll is" + str(phi*180/math.pi))
    print("pitch is" + str(theta*180/math.pi))
    print("yaw is" + str(psi*180/math.pi))
    print("\n")
    time.sleep(1)

