%Start with plane dynamics, add orbital effects later

%Step 1: Initialize states and properties

%Order of state vec: x,y,z,vx,vy,vz,thetax,thetay,thetaz,wx,wy,wz

current_state = zeros(1,12); %For now use absolute position instead of relative position, might wanna change that
state_mat = current_state; %will use to track states over time, add new state every iteration
time = 0;
dt = .01; %time step size

end_con = 0;
current_event = 1; %on the first event
event_complete = 0; %0 if currently working on finishing event, 1 if between states and completing attitude 
%adjustments or waiting for time requirements to be met
event_ending_time = []; %keep track of when events are completed in case the next event needs to wait a certain amount of 
%time before starting


I_sc = .067; %Build higher fidelity model later in which mass changes
m_sc = 3.4958;

%Step 2: build the event matrix - fpr now just one translation and one
%rotation

%First column is the variable to check against to determine if the event
%has started. Corresponds to the column in the state, but if it's 0 it's
%time

%Second column is the value to check against to determine if the event
%has started. If it's time, the value is time since the previous event
%ended (so keep track of the time each event ends)!

%Third column is the tolerance for the value above

%Forth column is the variable that's being changed

%Fifth column is the value of the variable reached that determines if the
%event is concluded (note that angles will be in degrees for now)

%Sixth column is the tolerance for the value above

event_mat = [1 0 .1 4 .5 .05; 1 9 1 4 0 .05];%; 0 2 dt 11 15 .1; 8 90 .1 11 0 .1]; %play around with the tolerances later. Also might wanna build this
%up row by row so I can comment each one individually

%Start with initial intended state vector, this will be added to as we get
%to new events
intended_state = current_state;
intended_state(4) = .5; %first change is move forwards in x direction

while end_con == 0
        
    %BEGIN EVENT LOGIC
if event_complete == 0 %If we're in the middle of an event
    
    change_var = event_mat(current_event, 4); %the variable we're changing, will use this to see if the event has ended

    if abs(current_state(change_var) - event_mat(current_event, 5)) < event_mat(current_event, 6)
        %If current state is within tolerance of ending condition for event
        event_complete = 1;
        force = [0 0 0]; %wait until next iteration to do anything with the thrusters, on time matrix also stays the same
        torque = [0 0 0];
        event_ending_time = [event_ending_time; time]; %logging the time in case the next event needs it
        
    else %We're still going with the event, keep activating control system to determine how to do manuever
        intended_state = current_state;
        intended_state(change_var) = event_mat(current_event, 5); %update the intended state to the current one
        %The only variable that changes is the one targeted in this event.
        %It will keep changing until it hits the target, which is a
        %velocity
%         current_state
%         intended_state
%         time
        [force, torque] = maneuver_control(current_state, intended_state);
    end
    
else % If not in the middle of the event
    
    %First check if the attitudes are good, if not call attitude control
    %and break out of this function
    
    angular_position_tol = .5; %At what point is it so off we care. This can easily change, is currently in degrees
    
    %Following lines perform attitude control, uncomment when function is
    %ready. Will definitely need some work
    
%     for column = 7:9 %just checking the angular positions, might wanna add angular rates
%         
%         %Note: as this gets more complex, might want to make the checking
%         %of if attitude needs to be changed its own function. But putting
%         %it here works for now
%         
%         if abs(current_state(column) - intended_state(column)) > tol %If attitude is too messed up
%             [force, torque, thruster_on_time] = attitude_maintenance_control(current_state, intended_state, thruster_on_time);
%              break
%         end
%         
%     end
    %At this point we've decided the attitude is good (once I finish that section of code). Nowcheck if the next
    %event has starrted
    force = [0 0 0]; torque = [0 0 0]; %Once again wait until the next iteration to do anything
    if current_event == length(event_mat(:,1))
        break
    end
    
    next_state_var = event_mat(current_event+1, 1);
    if abs(current_state(next_state_var) - event_mat(current_event+1, 2)) < event_mat(current_event, 3) %If conditions for next event are met
        current_event = current_event + 1;
        event_complete = 0; %we're back to the start of the next event
    end
    
end

%END EVENT LOGIC

    
    time = time + dt;


    %Numerical integration - make this next part a lot better. For now just
    %add velocity then add position (so new velocity affects position -
    %check this later)
    
     a = force/m_sc;
     alpha = torque/I_sc; 
     current_state(4:6) = current_state(4:6) + a*dt; %force is a vector
     current_state(10:12) = current_state(10:12) + alpha*dt; %want to change this with updated inertia matrix to include POI
     current_state(1:3) = current_state(1:3) + dt*current_state(4:6);
     current_state(7:9) = current_state(7:9) + dt*current_state(10:12);
     
     state_mat = [state_mat; current_state];
    if event_complete == 1 && current_event == length(event_mat(:,1))%Just complete the last event, you're done
        end_con == 1;
    end  
    
end

plot(0:dt:time, state_mat(:,1))
xlabel('time (seconds)')
ylabel('x position (m)')

function [force, torque] = maneuver_control(current_state, intended_state)
%Later step: log thruster on times

%Start off with simple function that only cares about the position, not the
%rates
thrust_mag = .5; %N
torque_mag = thrust_mag*.15*2; %update later

%The following line of code gives the mapping of delta v/ delta w and the
%thrusters used to execute that. Actually worry about this later
thruster_map = [1 5 6 2 1 3 3 1; 3 6 8 4 7 5 7 5]; 

state_diff = [intended_state(4:6)-current_state(4:6) intended_state(10:12)-current_state(10:12)];
loc = find(state_diff, 1); %find the first nonzero index (though there should only be one)

if loc < 4 %need to change a linear velocity
    torque = [0 0 0];
    force = [0 0 0];
    force(loc) = thrust_mag*state_diff(loc)/abs(state_diff(loc)); %1 or -1 * thrust magnitude
    
else %need to change an angular velocity
    force = [0 0 0];
    torque = [0 0 0];
    torque(loc-3) = torque_mag*state_diff(loc)/abs(state_diff(loc)); %1 or -1 * torque magnitude, subtract 3 to get to right index
end
end



