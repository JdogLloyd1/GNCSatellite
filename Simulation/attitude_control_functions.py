import math

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

def maneuver_control(current_state, intended_state, col, calc_accl, vel_tar):
    #col already has 1 subtracted so it's in 0 indexing. Will always be 0-2 for position
    
    torque = [0, 0, 0] #default value
    
    theta_dif = current_state[col] - intended_state[col]
    omega_dif = current_state[col+3] - intended_state[col+3]
    omega_cur = current_state[col + 3]
    
    if abs(theta_dif) < omega_cur**2/(2*calc_accl):
        #if how long you have to go will take less then than slowing down, then slow down
        
        if omega_cur == 0:
            omega_cur += .0001 #can't divide by zero
        
        torque[col] = -1*omega_cur/abs(omega_cur) #negative or positive 1 to slow it down
        #change line below to elif
    elif theta_dif > 0: #too far in positive direction and not close enough to slow down
        if omega_cur >= -vel_tar: #unless moving fast enough in negative direction, move backwards
            torque[col] = -1
    
    else: # too far rotated in negative direction and not close enough to slow down
        if omega_cur <= vel_tar: #unless moving fast enough in positive direction, move forwards
            torque[col] = 1
    
    return torque

def attitude_control(current_state, intended_state, col, omega_tar):
    #col is 0-5, can be position or rotation
    
    torque = [0, 0, 0] #default values
    
    if col < 3: #changing attitude position
    
        theta_dif = current_state[col] - intended_state[col]
        omega_dif = current_state[col+3] - intended_state[col+3]
        omega_cur = current_state[col + 3]
        
        if theta_dif > 0: #rotated too far in positive direction
            if omega_cur >= -omega_tar: #unless I'm moving fast enough in negative direction, rotate backwards
                torque[col] = -1
        else: #too far rotated in negative direction
            if omega_cur <= omega_tar: #unless I'm moving fast enough in positive direction, rotate forwards
                torque[col] = 1
                
    else: #changing angular rate
    
        theta_dif = current_state[col-3] - intended_state[col-3]
        omega_dif = current_state[col] - intended_state[col]
        
        if omega_dif > 0: #moving too fast in positive direction, slow down
            torque[col - 3] = -1 
        
        elif omega_dif < 0: #moving too fast in negative direction, accelerate in positive direction
            torque[col - 3] = 1
        
    return torque
                
                
            
        
    