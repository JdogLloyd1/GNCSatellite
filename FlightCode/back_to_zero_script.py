import math
import matplotlib.pyplot as plt
from attitude_control_functions import *
import time
import RPi.GPIO as GPIO
import traceback

import board
import busio
import adafruit_bno055 

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.mode = adafruit_bno055.NDOF_MODE

last_val = 0xFFFF
print("Quaternion: {}".format(sensor.quaternion))

def cleanAndExit():
    
    print("Exiting Flight Control")
    
    # turn off all relays
    GPIO.output(4, GPIO.LOW)
    GPIO.output(5, GPIO.LOW)
    GPIO.output(6, GPIO.LOW)
    GPIO.output(22, GPIO.LOW)
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(25, GPIO.LOW)
    GPIO.output(26, GPIO.LOW)

    GPIO.cleanup()

def sensorCalibration():
    while sensor.calibrated == False:

        print("Sys/Gyro/Acc/Mag")
        print(sensor.calibration_status)
        print(sensor.calibrated)
        print("")
    
    print("Calibration complete. Proceeding...")
    time.sleep(2)

## Next few lines are stuff for setup that will stay the same
time_step = 2 # 1/frequency, in seconds

end_con = 0 #binary variable that becomes 1 when final event is complete
current_event = 0  #add 1 for every new maneuver, starts at 0 bc that's the first index
event_complete = 0 #binary variable that = 0 when in maneuver and 1 when
#in between maneuvers
event_ending_time = [] #add time where each event ends

I_inv = [[1.4425, -.1697, -2.5*10**-4], [-.1697, 1.4425, 2.4*10**-4], \
         [-2.5*10**-4, -2.4*10**-4, 1.273]] #use this to estimate slow down time. UPDATE
    
#build the event matrix - format in documentation. Variables you're changing are 1-6, 
#first 3 are angular positions then angular velocities. Note they'r 1 above python indeces
event_mat =  [[0,.25,.1,3, math.pi/2,.01], [0,.25,.1,2,-math.pi/3,.05], [0,.25,.1,2,0,.05], [0,.25,.1,3,math.pi,.05] ]
              # do maneuvers below to get back to 0 at end, keep simple for now
              #0 .25 .1 8 0 .05; 0 .25 .1 9 pi/2 .

# Initialize state with IMU data
position = sensor.quaternion
euler_angles = convert_quaternion(position)
ang_vel = sensor.gyro

current_state = [euler_angles[0], euler_angles[1], euler_angles[2], ang_vel[0], ang_vel[1], ang_vel[2]]

intended_state = [0]*6 
change_var = event_mat[0][3] - 1 #the variable we're changing, -1 is so indeces are right
#intended_state[change_var] = event_mat[0][4]

num_attitude_checks = 1
            
state_check = [3, 4, 5, 0, 3, 1, 4, 2, 5]#, 0, 3, 1, 4, 2, 5] 
#order of attitude checks you do, putting in vector to allow for easy changing
#Fix the x, then the z, then the y.0 is x position, 3 is x velocity

thruster_mat = [0, 0] #track which thrusters turn on, might not need this         
 
# Set up relays
GPIO.setmode(GPIO.BCM)
# GPIO Pins
# 1 = GPIO4
# 2 = GPIO5
# 3 = GPIO6
# 4 = GPIO22
# 5 = GPIO23
# 6 = GPIO24
# 7 = GPIO25
# 8 = GPIO26

# Pin Setup
GPIO.setup(4, GPIO.OUT)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)

prev_valves = [0, 0]
valve_dict = {1:4, 2:5, 3:6, 4:22, 5:23, 6:24, 7:25, 8:26}

#sensorCalibration()

print("Commencing Flight Control")

#initialize lists for plotting
time_vec = []
roll_vec = []
pitch_vec = []
yaw_vec = []
yw_vec = []
wx_vec = []
wy_vec = []
wz_vec = []

## Long while loop is the heart of the control algorithm for control logic, won't change    
try:
    while num_attitude_checks < len(state_check):
        print("\n")
        
        loop_start_time = time.time()
        position = sensor.quaternion
        euler_angles = convert_quaternion(position)
        ang_vel = sensor.gyro
        
        print("roll is" + str(phi*180/math.pi))
        print("pitch is" + str(theta*180/math.pi))
        print("yaw is" + str(psi*180/math.pi))
        
        current_state = [euler_angles[0], euler_angles[1], euler_angles[2], ang_vel[0], ang_vel[1], ang_vel[2]]
            
        #case 3 of 4 is you've finished the event and slowed down but NOT all attitudes are fixed

        
        torque = [0, 0, 0] #default values
        
        theta_tol = 5*math.pi/180  #tolerance for position, number is in degrees
        omega_tol = 2*math.pi/180

        if num_attitude_checks < 10:
            omega_tar = 10*math.pi/180 #target rotational velocity during attitude position fixes
        else:
            omega_tar = 2*math.pi/180
        current_col = state_check[num_attitude_checks - 1]  
        
        if current_col < 3: #fixing an attitude position
            
            if abs(current_state[current_col] - intended_state[current_col]) > theta_tol:
                #Attitude is outside of tolerance, fix it
                
                torque = attitude_control(current_state, intended_state, current_col, omega_tar)
            
            else: #atttitude is good, move onto next one
            
                num_attitude_checks += 1
                
        elif current_col < 6: #fixing angular rate
        
            if abs(current_state[current_col] - intended_state[current_col]) > omega_tol:
                #Attitude is outside of tolerance, fix it
                
                torque = attitude_control(current_state, intended_state, current_col, omega_tar)
            
            else: #atttitude is good, move onto next one
            
                num_attitude_checks += 1            
                



                    
        ## End of event logic. Lines of code below won't change
        
        #figure out which thrusters to turn on based on torque -doesn't change
        #numerical integration stuff
        
        #figure out which thrusters to turn on based on torque -doesn't change
        #numerical integration stuff
        if torque[0] > 0: #positive roll
            thruster_pair = [2, 4] #1y, 2y
            print('MANEUVER IS + ROLL')
        elif torque[0] < 0: #negative roll
            thruster_pair = [6, 8] #3y, 4y
            print('MANEUVER IS - ROLL')
        elif torque[1] > 0: #positive pitch
            thruster_pair = [1, 7] #1x, 4x
            print('MANEUVER IS + PTICH')
        elif torque[1] < 0: #negative pitch
            thruster_pair = [3, 5] #2x, 3x
            print('MANEUVER IS - PITCH')
        elif torque[2] > 0: #positive yaw
            thruster_pair = [3, 7] #2x, 4x
            print('MANEUVER IS + YAW')
        elif torque[2] < 0: #negative yaw
            thruster_pair = [1, 5] #1x, 3x
            print('MANEUVER IS - YAW')
        else: #no thrust
            thruster_pair = [0, 0]
            print('HOLD')
        
        
        elapsed_time = time.time() - loop_start_time
        time.sleep(time_step-elapsed_time)
        
        # CALL RELAY FUNCTION TO TURN ON RELAYS AND VALVES
        if thruster_pair != prev_valves: #only turn on/off valves if it's different from the last timestep\
            
            for valve in prev_valves:
                if valve != 0:
                    GPIO.output(valve_dict[valve], GPIO.LOW)
                
            for valve in thruster_pair:
                if valve != 0:
                    GPIO.output(valve_dict[valve], GPIO.HIGH)
                
        prev_valves = [thruster_pair[0], thruster_pair[1]] #reset the loop
        
        #put together lists for plotting
        roll_vec.append(current_state[0]*180/math.pi)
        pitch_vec.append(current_state[1]*180/math.pi)
        yaw_vec.append(current_state[2]*180/math.pi)
        wx_vec.append(current_state[3]*180/math.pi)
        wy_vec.append(current_state[4]*180/math.pi)
        wz_vec.append(current_state[5]*180/math.pi)
        time_vec.append(time.time())
        
    plt.figure(1)
    #first figure is for angular positions

    plt.subplot(321)
    plt.plot(time_vec, roll_vec, label = 'Roll')
    leg = plt.legend()
    plt.subplot(323)
    plt.plot(time_vec, pitch_vec, label = 'Pitch')
    leg = plt.legend()
    plt.subplot(325)
    plt.plot(time_vec, yaw_vec, label = 'Yaw')
    leg = plt.legend()

    #plt.figure(2)
    #second figure is for angular velocities

    plt.subplot(322)
    plt.plot(time_vec, wx_vec, label = 'Omega x')
    leg = plt.legend()
    plt.subplot(324)
    plt.plot(time_vec, wy_vec, label = 'Omega y')
    leg = plt.legend()
    plt.subplot(326)
    plt.plot(time_vec, wz_vec, label = 'Omega z')
    leg = plt.legend()
    plt.show()

except:
    traceback.print_exc()
    print("Error occurred, cleaning up")
    cleanAndExit()
else:
    cleanAndExit()