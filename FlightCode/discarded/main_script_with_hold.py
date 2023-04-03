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
    cal_non_accl = False
    while sensor.calibrated == False and cal_non_accl == False:

        print("Sys/Gyro/Acc/Mag")
        print(sensor.calibration_status)
        print(sensor.calibrated)
        print("")
        time.sleep(.5)
        if sensor.calibration_status[0] == 3 and sensor.calibration_status[1] == 3 and sensor.calibration_status[3] == 3:
            cal_non_accl = True

sensorCalibration()
time.sleep(5)


## Next few lines are stuff for setup that will stay the same
time_step = 1 # 1/frequency, in seconds

end_con = 0 # binary variable that becomes 1 when final event is complete
current_event = 0  # add 1 for every new maneuver, starts at 0 bc that's the first index
event_complete = 0 # binary variable that = 0 when in maneuver and 1 when
# in between maneuvers
event_ending_time = [] # add time where each event ends

I_inv = [[1.4425, -.1697, -2.5*10**-4], [-.1697, 1.4425, 2.4*10**-4], \
         [-2.5*10**-4, -2.4*10**-4, 1.273]] # use this to estimate slow down time. UPDATE
    
# build the event matrix - format in documentation. Variables you're changing are 1-6, 
# first 3 are angular positions then angular velocities. Note they'r 1 above python indeces

event_mat =  [[0,.25,.1,3, math.pi/6,10*math.pi/180], [0,.25,.1,2,-math.pi/6,5*math.pi/180],\
              [0,10,.1,2,0,5*math.pi/180], [0,.25,.1,3,50*math.pi/180,10*math.pi/180]]
#[[0,.25,.1,3,1.25,.01], [0,.25,.1,2,-.517,.05], [0,.25,.1,2,0,.05], [0,.25,.1,3,1.5,.05] ]
              # do maneuvers below to get back to 0 at end, keep simple for now
              #0 .25 .1 8 0 .05; 0 .25 .1 9 pi/2 .

# Initialize state with IMU data
position = sensor.quaternion
euler_angles = convert_quaternion(position)
ang_vel = sensor.gyro

current_state = [euler_angles[0], euler_angles[1], euler_angles[2], ang_vel[0], ang_vel[1], ang_vel[2]]

# Set up other parameters needed to know where 
intended_state = [0]*6 
change_var = event_mat[0][3] - 1 #the variable we're changing, -1 is so indeces are right
intended_state[change_var] = event_mat[0][4]

direction = (intended_state[change_var] - current_state[change_var])\
    /abs(intended_state[change_var] - current_state[change_var])
            
state_check = [0, 3, 2,5, 1, 4] #,0, 3, 2, 5, 1, 4] 
# Order of attitude checks you do, putting in vector to allow for easy changing
# Fix the x, then the z, then the y.0 is x position, 3 is x velocity

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
    while end_con == 0:
        print("\n")
        #print([current_state[0]*180/math.pi, current_state[1]*180/math.pi, current_state[2]*180/math.pi])
        #print([intended_state[0]*180/math.pi, intended_state[1]*180/math.pi, intended_state[2]*180/math.pi])
        
        loop_start_time = time.time()
        position = sensor.quaternion
        euler_angles = convert_quaternion(position)
        ang_vel = sensor.gyro

        current_state = [euler_angles[0], euler_angles[1], euler_angles[2], ang_vel[0], ang_vel[1], ang_vel[2]]
        print("current event is " + str(current_event))
        if event_complete == 0: #case 1 of 4 is you're in the middle of a maneuver
        
            change_var = event_mat[current_event][3] - 1 #the variable we're changing, -1 is so indeces are right
            #typically a position

            #if statement below determines if you've exceed target val or within tolerance, 
            # if so changes event_complete. sligtly different eqn depending on direction
            if direction == 1:
                if current_state[change_var] > event_mat[current_event][4] or \
                abs(current_state[change_var] - event_mat[current_event][4]) < event_mat[current_event][5]:
                    event_complete = 1;
            else:
                if current_state[change_var] < event_mat[current_event][4] or \
                abs(current_state[change_var] - event_mat[current_event][4]) < event_mat[current_event][5]:
                    event_complete = 1;
                    
            if event_complete == 1: #passed conditions to end the maneuver
                
                torque = [0, 0, 0] #default value
                event_ending_time.append(time.time()) 
                num_attitude_checks = 0 #since event is done, we will recheck attitudes
                
            else: #maneuver not complete, continue calling control function
            
                vel_tar = 7.5*math.pi/180 #target angular speed is 15 deg/s
                calc_accl = .15*I_inv [change_var][change_var] 
                #use MOI to estimate time to slow down, ignore POI for this
                torque = maneuver_control(current_state, intended_state, change_var, calc_accl, vel_tar)
                

            print("in maneuver")
            
        elif event_complete == 1 and num_attitude_checks == 0: #case 2 of 4 is you're slowing down right after a maneuver ends
            
            vel_dif = current_state[change_var+3] - intended_state[change_var+3] 
            #difference between current and intended velocity
            vel_tol = 1*math.pi/180 #.02 #tolerance in rad/s
            torque = [0, 0, 0]
            
            if abs(vel_dif) > vel_tol:
                torque[change_var] = -vel_dif/abs(vel_dif) #.15 * 1 or -1, depending on direction
            else:
                num_attitude_checks = 1 #slowed down enough, move onto first attitude check
            print("slowing down")
            
        elif event_complete == 1 and num_attitude_checks < len(state_check)+1 and num_attitude_checks > 0\
            and current_event != 1:
            #case 3 of 4 is you've finished the event and slowed down but NOT all attitudes are fixed
            
            torque = [0, 0, 0] #default values
            
            tol_dict = {0:3*math.pi/180, 1:5*math.pi/180, 2:10*math.pi/180}
            omega_tol = 1*math.pi/180 #.1*math.pi/180
            omega_tar = 2*math.pi/180 #target rotational velocity during attitude fixes
            
            current_col = state_check[num_attitude_checks - 1]  
            change_var = current_col

            if current_col < 3: #fixing an attitude position
                theta_tol = tol_dict[current_col]

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
            print("num of attitude checks is " + str(num_attitude_checks))
            
        else: #Case 4/4: If event is done and and all attitudes have been fixed, see if next event has started
        
            torque = [0, 0, 0] #default values
            
            if current_event == len(event_mat) - 1: #no more events, so don't need the rest of while loop code
            #thrusters aren't turning on this step anyways. Subtract 1 bc current_event starts with 0 index
                break
            
            next_event_var = event_mat[current_event + 1][0] #variable to check if the next event has started
            
            if next_event_var == 0: #next event based on time
                if time.time() - event_ending_time[current_event] > event_mat[current_event+1][1]:
                    #If enough time has past since next event, set it up for next event
                    #Note that I'll have to change time to the actual time
                    
                    #update intended state to include the target value of the next event
                    change_var = event_mat[current_event + 1][3] - 1
                    intended_state[change_var] = event_mat[current_event + 1][4] 
                    
                    current_event += 1 #move onto next event
                    event_complete = 0 #back to start of next event
                    
                    direction = (intended_state[change_var] - current_state[change_var])\
                        /abs(intended_state[change_var] - current_state[change_var])
                        
                else:
                #if conditions are based on time and you've got more than .1 seconds left
                #perform a hold maneuver. This will look like an angular velocity auto correct
                # in pitch if it's outside of tolerance. .1 is just a hard coding way to specify
                #it's after the first pitch maneuver, will make sure all other wait times are below that
                     
                    elapsed_time = time - event_ending_time[len(event_ending_time)-1]
                    wait_time = event_mat[current_event+1][1]
                    if wait_time - elapsed_time > .25:
                        print("waiting for next event")
                        omega_tol = .5*math.pi/180 #play around with this
                        
                        #only care about pitch velocity
                        if abs(current_state[4] - intended_state[4]) > omega_tol:
                           #Attitude is outside of tolerance, fix it
                           current_col = 4 #fix pitch velocity
                           torque = attitude_control(current_state, intended_state, current_col, omega_tar) 
                        
            else: #conditions for next event aren't based on time
                
                if abs(current_state[next_event_var - 1] - event_mat[current_event+1][1]) < event_mat[current_event][2] :
                #start next event if you're within tolerance for the starting conditions
                
                    #same code as above except we wanna update intended state to the variable
                    #we just used to check and the ending point of the next maneuver
                    intended_state[next_event_var - 1] = event_mat[current_event+1][1]
                    
                    change_var = event_mat[current_event + 1][3] - 1
                    intended_state[change_var] = event_mat[change_var][4] 
                    
                    current_event += 1 #move onto next event
                    event_complete = 0 #back to start of next event
                    
                    direction = (intended_state[change_var] - current_state[change_var])\
                        /abs(intended_state[change_var] - current_state[change_var])
        

 
        if change_var == 0:
            print("Current roll is " + str(current_state[change_var]*180/math.pi) + " degrees, intended roll is " + str(intended_state[change_var]*180/math.pi))
        
        if change_var == 1:
            print("Current pitch is " + str(current_state[change_var]*180/math.pi) + " degrees, intended pitch is " + str(intended_state[change_var]*180/math.pi))
   
        if change_var == 2:
            print("Current yaw is " + str(current_state[change_var]*180/math.pi) + " degrees, intended yaw is " + str(intended_state[change_var]*180/math.pi))

        ## End of event logic. Lines of code below won't change
        
        #figure out which thrusters to turn on based on torque -doesn't change
        #numerical integration stuff
        if torque[0] > 0: #positive roll
            thruster_pair = [2, 4] #1y, 2y
            print('+ roll')
        elif torque[0] < 0: #negative roll
            thruster_pair = [6, 8] #3y, 4y
            print('- roll')
        elif torque[1] > 0: #positive pitch
            thruster_pair = [1, 7] #1x, 4x
            print('+ pitch')
        elif torque[1] < 0: #negative pitch
            thruster_pair = [3, 5] #2x, 3x
            print('- pitch')
        elif torque[2] > 0: #positive yaw
            thruster_pair = [3, 7] #2x, 4x
            print('+ yaw')
        elif torque[2] < 0: #negative yaw
            thruster_pair = [1, 5] #1x, 3x
            print('- yaw')
        else: #no thrust
            thruster_pair = [0, 0]
            print('hold')
        
        if event_complete == 1 and current_event == len(event_mat) - 1:
            end_con == 1 #if you're on the last event and completed it, you're done
           
        
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
