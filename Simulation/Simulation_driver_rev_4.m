%Current Inertia tensor is higher than expected and has high products in xy
%direction to test the sim, actual inertia tensor should be easier to work
%with

I = 20*[120.122 14.133 .021; 14.133 120.122 -.02; .021 -.02 134.255]*.0002926; 
I_inv = inv(I); 
    
%Step 1: Initialize states and properties

%Order of state vec: x,y,z,vx,vy,vz,thetax,thetay,thetaz,wx,wy,wz
%thetax, thetay, thetaz will be the z-y-x euler angles
%Note that angular velocities will be tracked in the frame of the satellite

current_state = zeros(1,12); 
current_state(1:3) = [-87.4312 -111.3507 -24.9441];  
%position vector from junk to sat

%The velocities we wanna hit are 0.7419 1.2245 0.2483;
state_mat = current_state; %will use to track states over time, add new state every iteration
time = 0;
dt = .01; %time step size

end_con = 0; %Binary variable, this becomes 1 when the final event is complete
current_event = 1; %Start on first event, add 1 for every new event
event_complete = 0; %Binary variable,0 if currently working on finishing an event, 1 if between states and completing attitude 
%adjustments or waiting for time requirements to be met
event_ending_time = []; %keep track of when events are completed in case the next event needs to wait a certain amount of 
%time before starting

m_sc = 3.4958; %Using mass of combined system

%constants for orbital effects
mu = 3.986*10^14;
rt = (6371+300)*1000; %convert to m
w = sqrt(mu/(rt^3));

%Step 2: build the event matrix 

%First column is the variable to check against to determine if the event
%has started. Corresponds to the column in the state, but if it's 0 it's
%time

%Second column is the value to check against to determine if the event
%has started. If it's time, the value is time since the previous event
%ended (tracked in event_ending_time).

%Third column is the tolerance for the value above

%Forth column is the variable that's being changed

%Fifth column is the value of the variable reached that determines if the
%event is concluded (note that angles will be in degrees for now)

%Sixth column is the tolerance for the value above

event_mat =  [0 .25 .1 9 1.0261 .01; 0 .25 .1 8 -.1717 .05; 0 0.25 .1 1 1 .01; 0 .25 .1 8 0 .05; 0 .25 .1 9 pi/2 .01];%; 0 0.25 .1 1 5 .01];
%Assume that the time for the rotational maneuvers is negligible compared
%to the time it waits after the translational maneuver
%Translation after final maneuver is being weird, need to fix that. Also
%check orbital effects are inputted correctly

%Start with initial intended state vector, this will be added to as we get
%to new events. Think of intended state as where you should be and current
%state is where you are. Both will be continually updated throughout the
%mission/sim

%Initialize intended state, current state, the variables of interest, and
%initial direction

intended_state = current_state;
%intended_state(9) = 1.0261; %first change is move forwards in x direction, manually adding first maneuver

change_var = event_mat(1, 4); %the variable we're changing
intended_state(change_var) = event_mat(1, 5);

direction = (intended_state(change_var) - current_state(change_var))...
    /abs(intended_state(change_var) - current_state(change_var));
            
state_check = [7 10 9 12 8 11 7 10 9 12 8 11 7 10]; %order of attitude checks you do, putting in vector to allow for easy changing
%Fix the x, then the z, then the y.

thruster_mat = [0 0]; %track which thrusters turn on

while end_con == 0
    
    %BEGIN EVENT LOGIC
    
if event_complete == 0 %If we're in the middle of an event
    
    change_var = event_mat(current_event, 4); %the variable we're changing, will use this to see if the event has ended.
    %This will typically be a position
    
    %Following if statements determine if you've exceeded the necessary
    %value for the event to be done.
    if direction == 1
        if current_state(change_var) > event_mat(current_event, 5) | ... %you've passed target position or you're within tolerance
                abs(current_state(change_var) - event_mat(current_event, 5)) < event_mat(current_event, 6)
            event_complete = 1;
        end
    else
        if current_state(change_var) < event_mat(current_event, 5) | ... %you've passed target position or you're within tolerance
                abs(current_state(change_var) - event_mat(current_event, 5)) < event_mat(current_event, 6)
            event_complete = 1;
        end
    end
    
    if event_complete == 1
        %If you've passed the ending conditions for the event
        
        force = [0 0 0]; %wait until next iteration to do anything with the thrusters, it'll keep things simpler
        torque = [0 0 0];
        event_ending_time = [event_ending_time; time]; %logging the time in case the next event needs it
        num_attitude_checks = 0; %next step is to slow down completely, then check attitudes. This keeps track of 
        %how many attitude variables we've corrected/verified are good
        
    else %We're still going with the event, keep activating control system to determine how to do manuever
        
        if change_var < 7 %linear change
            vel_tar = .742; %want to translate at .5 m/s
            %Add a line of hardcoding here to get to different targets for
            %different maneuvers
            calc_accl = 1/m_sc; %force mag = 1 N (2x .5N thrusters). Update with experimental data!
            %Estimating the acceleration so we can use kinematics to
            %determine where to start slowing down
        else
            vel_tar = 15*pi/180; %want to rotate at 15 deg/s
            calc_accl = .15*I_inv(change_var-6, change_var-6); %torque = .15 Nm, alpha = torque*I inverse,
            %Estimating the acceleration so we can use kinematics to
            %determine where to start slowing down
        end
            
        [force, torque] = maneuver_control(current_state, intended_state, change_var, calc_accl, vel_tar);
        %Function included at the bottom, uses the difference between
        %current and intended state to determine which thrusters to fire
    end
    
elseif event_complete == 1 && num_attitude_checks ==  0
    %You just finished the event and need to slow the spacecraft down
    
    %If velocity difference is in tolerance, do nothing and start doing atttitude checks.
    %If not, slow down. 
    
    vel_dif = (-intended_state(change_var+3) + current_state(change_var+3)); 
    %Works as long as maneuver is just to change position, which shouldn't
    %change
    %Positive means you're over what you're supposed to be
    
    force = [0 0 0]; %default values
    torque = [0 0 0];
    
    if change_var < 7 %previous maneuver was translation
        dir = change_var;
        vel_tol = .01;
        if abs(vel_dif) > vel_tol %moving too fast in one direction
            force(dir) = -vel_dif/abs(vel_dif); %accelerate in opposite direction of velocity to slow down
        else
            num_attitude_checks = 1; %you've slowed down enough, move onto attitude checks
        end
    else %Previous maneuver was rotation
        dir = change_var - 6;
        vel_tol = .02; %Angular velocity tolerance
        if abs(vel_dif) > vel_tol %moving too fast in one direction
            torque(dir) = -.15*vel_dif/abs(vel_dif); %accelerate in opposite direction of velocity to slow down
        else
            num_attitude_checks = 1; %you've slowed down enough, move onto attitude checks
        end
    end 
    
elseif event_complete == 1 && num_attitude_checks < length(state_check) && num_attitude_checks > 0 
    % If not in the middle of the event AND not all attitudes/rates are
    % good, but you have fully slowed down
    
    force = [0 0 0]; torque = [0 0 0]; %default values
   
    theta_tol = .2*pi/180; %converting to radians, these are easy things to use for tuning
    omega_tol = .1*pi/180; 
   
    current_col = state_check(num_attitude_checks); %getting which variable is being checked/fixed
    
    if current_col < 10 %If you're fixing an attitude position
        if abs(current_state(current_col) - intended_state(current_col)) > theta_tol 
        %If attitude is too messed up 
        
            [force, torque] = attitude_control(current_state, intended_state, current_col);
           
        else %Attitude position is good, can move onto the next one
            num_attitude_checks = num_attitude_checks + 1;
        end
        
    elseif current_col < 13 %If you're fixing an angular velocity
        if abs(current_state(current_col) - intended_state(current_col)) > omega_tol %angular velocity is messed up
            
            [force, torque] = attitude_control(current_state, intended_state, current_col);
            
            else %Attitude position is good, can move onto the next one
                num_attitude_checks = num_attitude_checks + 1; %when this gets past 12 we're done with attitude maintenance
        end
    end
    
    
else %If we're not in an event and all the attitudes have been verified to be good, see if the next event has started
    %If next event has started, wait until next timestep to start event,
    %turn on thrusters, etc to keep things simple
    
    force = [0 0 0]; torque = [0 0 0]; %default values. If not enough time has passed before the next event this will stay 0
    
    if current_event == length(event_mat(:,1)) %If there's no more events then you're done
        break
    end
    
    next_event_var = event_mat(current_event+1, 1); %Variable to check if the next event has started
    
    if next_event_var == 0 %If the next condition is based on time
        if time - event_ending_time(end) > event_mat(current_event, 3)
            %Checking if enough time has passed
            
            % Update intended state to include the target value of the next event
            change_var = event_mat(current_event+1, 4); %the variable we're changing
            intended_state(change_var) = event_mat(current_event+1, 5);
            
            current_event = current_event + 1;
            event_complete = 0; %we're back to the start of the next event
            direction = (intended_state(change_var) - current_state(change_var))...
                /abs(intended_state(change_var) - current_state(change_var));
            %direction is 1 if next move is in the positive direction, -1
            %if next move is in the negative direction
          
        end
        
    else  %If conditions for next event aren't based on time
        
        if abs(current_state(next_event_var) - event_mat(current_event+1, 2)) < event_mat(current_event, 3) 
           %If conditions for the start of the next event have been met
            
            check_var = event_mat(current_event+1, 1); %the variable we're checking against gives position of next state
            intended_state(check_var) = event_mat(current_event+1, 2); 
            
            change_var = event_mat(current_event+1, 4); %the variable we're changing
            intended_state(change_var) = event_mat(current_event+1, 5);
            %The updated value you're targeting is in event mat, and that's
            %what your intended state becomes
            
            intended_state;
            current_state;
            current_event = current_event + 1;
            event_complete = 0; %we're back to the start of the next event
            
            direction = (intended_state(change_var) - current_state(change_var))...
                /abs(intended_state(change_var) - current_state(change_var));
        end
    end
    
end

%END EVENT LOGIC

    %Numerical integration - for now is simple Forward Euler time stepping
    %scheme
    
    time = time + dt;

    %Accelerations to velocities numerical integration is simple because
    %both are in the frame of the satellite
    
    %Numerical integration from velocities to positions requires a rotation
    %matrix due to the velocities being tracked in the frame of the
    %satellite
    
    alpha = current_state(9); %yaw
    beta = current_state(8); %pitch
    gamma = current_state(7); %roll
    
    %The following 3 lines of code give a rotation matrix for z-y-x euler
    %angles. This is used for transforming linear velocities
    %actually don't worry about linear velocities for now, that should be
    %done in the frame of the junk entirely so the rotation matrix should
    %be for acceleration -> velocity, where velocity is in junk frame (but
    %angular velocities are in satellite frame). Also note that should
    %multiply individual rotation matrices to do that
    
    rot_mat = [cos(alpha)*cos(beta) cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma) cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma); ...
        sin(alpha)*cos(beta) sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma) sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma); ...
        -sin(beta) cos(beta)*sin(gamma) cos(beta)*cos(gamma)];      

    %The following lines give the rotation matrices for x and y-x
    %rotations. The x rotation matrix is applied to the pitch velocity, and
    %the y-x matrix is applied to the yaw velocity
    
    
    %vel_junk_frame = (rot_mat*current_state(4:6)')'; %don't worry about
    %linear stuff for now
    %omega_junk_frame = (rot_mat*current_state(10:12)')';
    
     a = force/m_sc;
     ang_accl = torque*I_inv;
 
    rot_mat_x = [1 0 0; 0 cos(gamma) -sin(gamma); 0 sin(gamma) cos(gamma)];
    rot_mat_y = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
    rot_mat_z = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
    rot_mat_yx = rot_mat_y*rot_mat_x;
    rot_mat_zyx = rot_mat_z*rot_mat_y*rot_mat_x;
    
     %acceleration to velocity integration
     current_state(4:6) = current_state(4:6) + (rot_mat_zyx*a'*dt)'; %all linear stuff is in the frame of the junk except the force
     orbit_accl = [2*w*current_state(5) -2*w*current_state(4)+3*w^2*current_state(5) -w^2*current_state(6)]; %calc orbital effects
     %add orbital effects 
     current_state(4:6) = current_state(4:6) + dt*orbit_accl;
     
     current_state(10:12) = current_state(10:12) + ang_accl*dt; 
     
     %linear velocities to linear accelerations integration
     current_state(1:3) = current_state(1:3) + dt*current_state(4:6);
     
          
    %velocity to position/euler angles integration    
    omega = current_state(10:12)';
    
    gamma = gamma + omega(1)*dt; %adding component from x velocity
    
    y_delta = dt*rot_mat_yx*[0; omega(2); 0]; %breaking local y velocity down into components
    gamma = gamma + y_delta(1);
    beta = beta + y_delta(2);
    alpha = alpha + y_delta(3);
    
    z_delta = dt*rot_mat_yx*[0; 0; omega(3)]; %adding components from y velocity
    gamma = gamma + z_delta(1);
    beta = beta + z_delta(2);
    alpha = alpha + z_delta(3);
     
    current_state(7) = gamma;
    current_state(8) = beta;
    current_state(9) = alpha;
    
    state_mat = [state_mat; current_state];
     
    if event_complete == 1 && current_event == length(event_mat(:,1)) %If you've completed the last event, 
        end_con == 1;
    end  
    
    %If statements to determine which thrusters turn on for the force or
    %torque
    if force == [0 0 0] %torque maneuvers
        if torque(1) > 0 %Positive roll
            thruster_pair = [2 4]; %1y, 2y
        elseif torque(1) < 0 %Negative roll
            thruster_pair = [6 8]; %3y, 4y
        elseif torque(2) > 0 %Positive pitch
            thruster_pair = [1 7]; %1x, 4x
        elseif torque(2) < 0 %Negative pitch
            thruster_pair = [3 5]; %2x, 3x
        elseif torque(3) > 0 %Positive yaw
            thruster_pair = [3 7]; %2x, 4x
        elseif torque(3) < 0 %Negative yaw
            thruster_pair = [1 5]; %1x, 3x
        end   
    elseif torque == [0 0 0] %force maneuvers
        if force(1) > 0 %positive x translation
            thruster_pair = [5 7]; %3x, 4x
        elseif force(1) < 0 %negative x translation
            thruster_pair = [1 3]; %1x, 2x
        elseif force(2) > 0
            thruster_pair = [2 8]; %1y, 4y
        elseif force(2) < 0
            thruster_pair = [4 6]; %2y, 3y
        end
    else
        thruster_pair = [0 0];
    end
    thruster_mat = [thruster_mat; thruster_pair];
    
end
thruster_mat
% PLOTS

%First set is linear positions and velocities
figure (1)
title('Linear Postions and Velocities')
time_vec = linspace(0, time, length(state_mat(:,1)));

subplot(3,2,1)
plot(time_vec, state_mat(:,1))
xlabel('time (seconds)')
ylabel('x position (m)')

subplot(3,2,2)
plot(time_vec, state_mat(:,4))
xlabel('time (seconds)')
ylabel('x velocity (m/s)')

subplot(3,2,3)
plot(time_vec, state_mat(:,2))
xlabel('time (seconds)')
ylabel('y position (m)')

subplot(3,2,4)
plot(time_vec, state_mat(:,5))
xlabel('time (seconds)')
ylabel('y velocity (m/s)')

subplot(3,2,5)
plot(time_vec, state_mat(:,3))
xlabel('time (seconds)')
ylabel('z position (m)')

subplot(3,2,6)
plot(time_vec, state_mat(:,6))
xlabel('time (seconds)')
ylabel('z velocity (m/s)')


%Figure 2: Angular positions and angular velocities
figure (2)
title('Angular Positions and Velocities')

subplot(3,2,1)
plot(time_vec, (180/pi)*state_mat(:,7))
xlabel('time (seconds)')
ylabel('x angular position (deg)')

subplot(3,2,2)
plot(time_vec, (180/pi)*state_mat(:,10))
xlabel('time (seconds)')
ylabel('x angular velocity (deg/s)')

subplot(3,2,3)
plot(time_vec, (180/pi)*state_mat(:,8))
xlabel('time (seconds)')
ylabel('y angular position (deg)')

subplot(3,2,4)
plot(time_vec, (180/pi)*state_mat(:,11))
xlabel('time (seconds)')
ylabel('y angular velocity (deg/s)')

subplot(3,2,5)
plot(time_vec, (180/pi)*state_mat(:,9))
xlabel('time (seconds)')
ylabel('z angular position (deg)')

subplot(3,2,6)
plot(time_vec, (180/pi)*state_mat(:,12))
xlabel('time (seconds)')
ylabel('z angular velocity (deg/s)')

%toc

function [force, torque] = maneuver_control(current_state, intended_state, col, calc_accl, vel_tar)
%Col is the variable you're changing, 1-3 are for linear position and
%7-9 are for angular position. Currently doesn't support just changing the
%velocity

%calc_accl is the calculated acceleration (m/s^2 or rad/s^2), used to
%determine when to slow down
%velocity_tar is the linear or angular velocity you want to move/rotate at

%Function to do the logic behind controlling maneuvers
%Later step: log thruster on times

thrust_mag = .5*2; %N, use two thrusters at once
torque_mag = thrust_mag*.15; 

%Assign default values of no thruster fires, change if one is needed
force = [0 0 0];
torque = [0 0 0];

if col < 4 %changing the linear position
    dir = col;
    pos_dif = (-intended_state(col) + current_state(col)); %positive means we're over what we want
    vel_dif = (-intended_state(col+3) + current_state(col+3));
    vel_cur = current_state(col+3);
    
    if abs(pos_dif) < vel_cur^2/(2*calc_accl) 
        %If you're close enough to the target position that if you don't
        %start slowing down you'll overshoot it, deaccelerate
        
        if vel_cur == 0
            vel_cur = vel_cur + .0001; %can't divide by 0
        end
        
        force(dir) = -thrust_mag*vel_cur/abs(vel_cur); %- or 1 * thrust magnitude
        
    elseif pos_dif > 0 %you're too far in the positive direction, but not close enough to slow down
        if vel_cur >= -vel_tar %unless you're moving fast enough in negative direction, move backwards
            force(dir) = -thrust_mag;
        end
        
    else %too far rotated in negative direction, but not close enough to slow down
        if vel_cur <= vel_tar %unless you're moving fast enough in positive direction, move forwards
            force(dir) = thrust_mag;
        end
    end
        
else %need the change the angular position. Logic is same as above. Note that all math is done in radians
    dir = col-6;
    theta_dif = (-intended_state(col) + current_state(col)); %positive means we're over what we want
    omega_dif = (-intended_state(col+3) + current_state(col+3));
    omega_cur = current_state(col+3);
    
    if abs(theta_dif) < omega_cur^2/(2*calc_accl) 
        %If how long you have to go is less than how long it takes to slow
        %down, slow down
        
        if omega_cur == 0
            omega_cur = omega_cur + .0001; %can't divide by 0
        end
        
        torque(dir) = -torque_mag*omega_cur/abs(omega_cur); %- or 1 * thrust magnitude to slow it down
        
    elseif theta_dif > 0 %you're too far in the positive direction, but not close enough to slow down
        if omega_cur >= -vel_tar %unless I'm moving fast enough in negative direction, move backwards
            torque(dir) = -torque_mag;
        end
        
    else %too far rotated in negative direction, but not close enough to slow down
        if omega_cur <= vel_tar %unless I'm moving fast enough in positive direction, move forwards
            torque(dir) = torque_mag;
        end
    end
    
end
end

function [force, torque] = attitude_control(current_state, intended_state, col)
%Function to get the attitude back to targeted state between maneuvers

force = [0 0 0]; %Not doing any translations here 
torque = [0 0 0]; %initialization, will stay like this if we're on the right course at the moment

thrust_mag = .5*2; %N, use two thrusters at once
torque_mag = thrust_mag*.15; 

%dir is the direction that we're fixing, is 1 for x, 2 for y, 3 for z
%col is the column of the state vector that needs to be fixed, for example
%y position is 8 and y velocity is 11

omega_tar = 2*pi/180; %target speed to do corrections is 2 deg/s. Good thing to tune

if col < 10 %changing the attitude position
    dir = col-6;
    theta_dif = (-intended_state(col) + current_state(col)); %positive means we're over what we want
    omega_dif = (-intended_state(col+3) + current_state(col+3));
    omega_cur = current_state(col+3);

    if theta_dif > 0 %too far rotated in the positive direction
        if omega_cur >= -omega_tar %unless I'm moving fast enough in negative direction, rotate backwards
            torque(dir) = -torque_mag;
        end
        
    else %too far rotated in negative direction
        if omega_cur <= omega_tar %unless I'm moving fast enough in positive direction, rotate forwards
            torque(dir) = torque_mag;
        end
    end
        
else %need the change the angular rate
    dir = col-9;
    theta_dif = (-intended_state(col-3) + current_state(col-3)); %positive means we're over what we want
    omega_dif = (-intended_state(col) + current_state(col));
    omega_cur = current_state(col)*180/pi;
    
    if omega_dif > 0 %moving too fast in positive direction, slow down
        torque(dir) = -torque_mag; 
        
    elseif omega_dif < 0 %moving too fast in negative direction, accelerate in positive direction
        torque(dir) = torque_mag;
    end
    
end

   
end