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

def make_omega_local(phi, theta, psi, omega):
    #turns the IMU's angular velocity vector into local coordinates
    #phi is roll, theta is pitch, psi is yaw, omega is gloabl angular velocity
    
    #general formula is to undo rotations: rotate -roll, then -pitch, then -yaw
    rot_mat_x = [[1,0,0], [0, math.cos(-phi), -math.sin(-phi)], [0, math.sin(-phi), math.cos(-phi)]]
    rot_mat_y = [[math.cos(-theta), 0, math.sin(-theta)], [0,1,0], [-math.sin(-theta), 0, math.cos(-theta)]]
    rot_mat_z = [[math.cos(-psi), -math.sin(-psi), 0], [math.sin(-psi), math.cos(-psi), 0], [0, 0, 1]] 
    rot_mat_xy = matrix_mult(rot_mat_x, rot_mat_y)
    rot_mat_xyz = matrix_mult(rot_mat_xy, rot_mat_z)
    omega_mat = matrix_mult(rot_mat_xyz, [[omega[0]], [omega[1]], [omega[2]]])
    omega_local = [omega_mat[0][0], omega_mat[1][0], omega_mat[2][0]]
    
    return omega_local
    
def convert_quaternion(position):

    q0 = position[0]
    q1 = position[1]
    q2 = position[2]
    q3 = position[3]

    if isinstance(q0, type(None)) or isinstance(q1, type(None)) or isinstance(q2, type(None)) or isinstance(q3, type(None)):
        #if IMU outputs nonetype because it sucks, output a specific piece of nonsense that tells the control fn to skip the timestep
        euler_angles = [1000, 1000, 1000]
        return euler_angles
        
    else:
        phi = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1**2 + q2**2)) #roll rotation

        #Have two options for pitch, use whichever works numerically
        inside_fun = 2*(q0*q2 - q1*q3)
        if inside_fun > 1:
            inside_fun = 1
        elif inside_fun < -1:
            inside_fun = -1
            
        #if abs(inside_fun) < 1:
        theta = math.asin(inside_fun)
        #else:
         #   print(position)
          #  theta = -math.pi/2 + 2*math.atan2(math.sqrt(1+2*(q0*q2 - q1*q3)), math.sqrt(1-2*(q0*q2-q1*q3))) #pitch rotation
            
        psi = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2**2 + q3**2)) #yaw rotation

        euler_angles = [phi, theta, psi]
        #print(theta)
        return euler_angles


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
                
            
        
    
