import math
import matplotlib.pyplot as plt
from attitude_control_functions import *

## Next few lines are stuff for setup that gets completely
# overwritten when we move away from the sim

m_sc = 3.4958
current_state = [0]*6 #replace with IMU data
state_mat = [] #for plotting, but IMU algorithm might need to track this so
# might actually be necessary
time = 0 #need to do something regarding time but prob different from this
dt = .01
time_vec = []
roll_vec = []
pitch_vec = []
yaw_vec = []
yw_vec = []
wx_vec = []
wy_vec = []
wz_vec = []
## Next few lines are stuff for setup that will stay the same

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

intended_state = [0]*6 
change_var = event_mat[0][3] - 1 #the variable we're changing, -1 is so indeces are right
intended_state[change_var] = event_mat[0][4]

direction = (intended_state[change_var] - current_state[change_var])\
    /abs(intended_state[change_var] - current_state[change_var])
            
state_check = [0, 3, 2, 5, 1, 4, 0, 3, 2, 5, 1, 4] 
#order of attitude checks you do, putting in vector to allow for easy changing
#Fix the x, then the z, then the y.0 is x position, 3 is x velocity

thruster_mat = [0, 0] #track which thrusters turn on, might not need this         
 

## Long while loop is the heart of the control algorithm for control logic, won't change    
          
while end_con == 0:

    if current_state[5] > 15*math.pi/180:
        nt = 1
    
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
            event_ending_time.append(time) 
            num_attitude_checks = 0 #since event is done, we will recheck attitudes
            
        else: #maneuver not complete, continue calling control function
        
            vel_tar = 15*math.pi/180 #target angular speed is 15 deg/s
            calc_accl = .15*I_inv [change_var][change_var] 
            #use MOI to estimate time to slow down, ignore POI for this
            torque = maneuver_control(current_state, intended_state, change_var, calc_accl, vel_tar)
            

        
    elif event_complete == 1 and num_attitude_checks == 0: #case 2 of 4 is you're slowing down right after a maneuver ends
        
        vel_dif = current_state[change_var+3] - intended_state[change_var+3] 
        #difference between current and intended velocity
        vel_tol = .02 #tolerance in rad/s
        torque = [0, 0, 0]
        
        if abs(vel_dif) > vel_tol:
            torque[change_var] = -vel_dif/abs(vel_dif) #.15 * 1 or -1, depending on direction
        else:
            num_attitude_checks = 1 #slowed down enough, move onto first attitude check
    
    elif event_complete == 1 and num_attitude_checks < len(state_check) and num_attitude_checks > 0:
        #case 3 of 4 is you've finished the event and slowed down but NOT all attitudes are fixed
        
        torque = [0, 0, 0] #default values
        
        theta_tol = .2*math.pi/180  #tolerance for position, number is in degrees
        omega_tol = .1*math.pi/180
        omega_tar = 2*math.pi/180 #target rotational velocity during attitude fixes
        
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
                
    else: #Case 4/4: If event is done and and all attitudes have been fixed, see if next event has started
    
        torque = [0, 0, 0] #default values
        
        if current_event == len(event_mat) - 1: #no more events, so don't need the rest of while loop code
        #thrusters aren't turning on this step anyways. Subtract 1 bc current_event starts with 0 index
            break
        
        next_event_var = event_mat[current_event + 1][0] #variable to check if the next event has started
        
        if next_event_var == 0: #next event based on time
            if time - event_ending_time[len(event_ending_time) - 1] > event_mat[current_event][2]:
                #If enough time has past since next event, set it up for next event
                #Note that I'll have to change time to the actual time
                
                #update intended state to include the target value of the next event
                change_var = event_mat[current_event + 1][3] - 1
                intended_state[change_var] = event_mat[current_event + 1][4] 
                
                current_event += 1 #move onto next event
                event_complete = 0 #back to start of next event
                
                direction = (intended_state[change_var] - current_state[change_var])\
                    /abs(intended_state[change_var] - current_state[change_var])
                    
                print(time)
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
                    
    ## End of event logic. Lines of code below won't change
    
    #figure out which thrusters to turn on based on torque -doesn't change
    #numerical integration stuff
    
    if torque[0] > 0: #positive roll
        thruster_pair = [2, 4] #1y, 2y
    elif torque[0] < 0: #negative roll
        thruster_pair = [6, 8] #3y, 4y
    elif torque[1] > 1: #positive pitch
        thruster_pair = [1, 7] #1x, 4x
    elif torque[1] < 1: #negative pitch
        thruster_pair = [3, 5] #2x, 3x
    elif torque[2] > 1: #positive yaw
        thruster_pair = [3, 7] #2x, 4x
    elif torque[2] < 1: #negative yaw
        thruster_pair = [1, 5] #1x, 3x
    else: #no thrust
        thruster_pair = [0, 0]
    
    if event_complete == 1 and current_event == len(event_mat) - 1:
        end_con == 1 #if you're on the last event and completed it, you're done
       
    
    ##numerical integration steps that I won't need once we move away from that
    time += dt
    
    alpha = current_state[2] #yaw
    beta = current_state[1] #pitch
    gamma = current_state[0] #roll
    
    rot_mat_x = [[1,0,0], [0, math.cos(gamma), -math.sin(gamma)], [0, math.sin(gamma), math.cos(gamma)]]
    rot_mat_y = [[math.cos(beta), 0, math.sin(beta)], [0,1,0], [-math.sin(beta), 0, math.cos(beta)]]
    rot_mat_z = [[math.cos(alpha), -math.sin(alpha), 0], [math.sin(alpha), math.cos(alpha), 0], [0, 0, 1]] 
    rot_mat_yx = matrix_mult(rot_mat_y, rot_mat_x)
    
    #numerical integration of acceleration to velocity
    
    ang_accl = matrix_mult([torque], I_inv) #have to make the torque vector a matrix
    current_state[3] += dt*ang_accl[0][0]*.15 #scale by the magnitude
    current_state[4] += dt*ang_accl[0][1]*.15
    current_state[5] += dt*ang_accl[0][2]*.15
    
    #numerical integration of velocity to position
    gamma += dt*current_state[3]
    
    y_delta = matrix_mult(rot_mat_yx, [[0], [current_state[4]],[0] ])
    gamma += dt*y_delta[0][0] #scale by timestep
    beta += dt*y_delta[1][0]
    alpha += dt*y_delta[2][0]
    
    z_delta = matrix_mult(rot_mat_yx, [[0], [0], [current_state[5]]])
    gamma += dt*z_delta[0][0] #scale by timestep
    beta += dt*z_delta[1][0]
    alpha += dt*z_delta[2][0]
    
    current_state[2] = alpha
    current_state[1] = beta
    current_state[0] = gamma
    
    roll_vec.append(current_state[0]*180/math.pi)
    pitch_vec.append(current_state[1]*180/math.pi)
    yaw_vec.append(current_state[2]*180/math.pi)
    wx_vec.append(current_state[3]*180/math.pi)
    wy_vec.append(current_state[4]*180/math.pi)
    wz_vec.append(current_state[5]*180/math.pi)
    time_vec.append(time)
    
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

    
