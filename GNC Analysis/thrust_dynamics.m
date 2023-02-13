%Corner Thrusters
r1 = [1 -1 -1]; %- [.1 -.05 .03];
r2 = [1 1 1]; %- [.1 -.05 .03];
r3 = [-1 1 -1]; %- [.1 -.05 .03];
r4 = [-1 -1 1]; %- [.1 -.05 .03];

f1x = [-1 0 0]; %these two thrusters have the same force vector
f1y = [0 1 0];
f1z = [0 0 1];

f2x = [-1 0 0];
f2y = [0 -1 0];
f2z = [0 0 -1];

f3x = [1 0 0];
f3y = [0 -1 0];
f3z = [0 0 1];

f4x = [1 0 0];
f4y = [0 1 0];
f4z = [0 0 -1];

t1x = cross(r1, f1x);
t1y = cross(r1, f1y);
t1z = cross(r1, f1z);

t2x = cross(r2, f2x);
t2y = cross(r2, f2y);
t2z = cross(r2, f2z);

t3x = cross(r3, f3x);
t3y = cross(r3, f3y);
t3z = cross(r3, f3z);

t4x = cross(r4, f4x);
t4y = cross(r4, f4y);
t4z = cross(r4, f4z);

force_mat = [f1x; f1y; f1z; f2x; f2y; f2z; f3x; f3y; f3z; f4x; f4y; f4z];
torque_mat = [t1x; t1y; t1z; t2x; t2y; t2z; t3x; t3y; t3z; t4x; t4y; t4z];

torque_result_mat = []; %Matrix of combos that give one torque with no forces
force_result_mat = []; %Matrix of combos that give one force with no torques

for i = 1:length(torque_mat(:,1))-1
    %disp(strcat("i is", num2str(i)))
    for j = (i+1):length(torque_mat(:,1)) 
       % disp(strcat("j is", num2str(j)))
       torque = torque_mat(i,:) + torque_mat(j,:);
       force = force_mat(i,:) + force_mat(j,:);
       
       if nnz(torque) == 1 && nnz(force) == 0 %check which combos give you a torque in one direction and no force
            torque_result_mat = [torque_result_mat; [i j torque]];
       end
       
       if nnz(torque) == 0 && nnz(force) == 1 %check which combos give you a force in one direction and no torque
            force_result_mat = [force_result_mat; [i j force]];
       end
       
    end
end

force_result_mat
torque_result_mat

%% SECTION 2: Do the same thing for thrusters on the edges
r1 = [1 0 -1];
r2 = [1 1 0];
r3 = [1 0 1];
r4 = [1 -1 0];
r5 = [0 1 -1];
r6 = [0 1 1];
r7 = [0 -1 1];
r8 = [0 -1 -1];
r9 = [-1 0 -1];
r10 = [-1 1 0];
r11 = [-1 0 1];
r12 = [-1 -1 0];

f1x = [-1 0 0];
f1z = [0 0 1];
f2x = [-1 0 0];
f2y = [0 -1 0];
f3x = [-1 0 0];
f3z = [0 0 -1];
f4x = [-1 0 0];
f4y = [0 1 0];

f5y =[0 -1 0];
f5z = [0 0 1];
f6y = [0 -1 0];
f6z = [0 0 -1];
f7y = [0 1 0];
f7z = [0 0 -1];
f8y = [0 1 0];
f8z = [0 0 1];

f9x = [1 0 0];
f9z = [0 0 1];
f10x = [1 0 0];
f10y = [0 -1 0];
f11x = [1 0 0];
f11z = [0 0 -1];
f12x = [1 0 0];
f12y = [0 1 0];
    
t1x = cross(r1, f1x);
t1z = cross(r1, f1z);
t2x = cross(r2, f2x);
t2y = cross(r2, f2y);
t3x = cross(r3, f3x);
t3z = cross(r3, f3z);
t4x = cross(r4, f4x);
t4y = cross(r4, f4y);

t5y = cross(r5, f5y);
t5z = cross(r5, f5z);
t6y = cross(r6, f6y);
t6z = cross(r6, f6z);
t7y = cross(r7, f7y);
t7z = cross(r7, f7z);
t8y = cross(r8, f8y);
t8z = cross(r8, f8z);

t9x = cross(r9, f9x);
t9z = cross(r9, f9z);
t10x = cross(r10, f10x);
t10y = cross(r10, f10y);
t11x = cross(r11, f11x);
t11z = cross(r11, f11z);
t12x = cross(r12, f12x);
t12y = cross(r12, f12y);

force_mat = [f1x; f1z; f2x; f2y; f3x; f3z; f4x; f4y; f5y; f5z; f6y; f6z; f7y; f7z; f8y; f8z; f9x; f9z; f10x; f10y; f11x; f11z; f12x; f12y];
torque_mat = [t1x; t1z; t2x; t2y; t3x; t3z; t4x; t4y; t5y; t5z; t6y; t6z; t7y; t7z; t8y; t8z; t9x; t9z; t10x; t10y; t11x; t11z; t12x; t12y];

torque_result_mat_edge = []; %Matrix of combos that give one torque with no forces
force_result_mat_edge = []; %Matrix of combos that give one force with no torques

for i = 1:length(torque_mat(:,1))-1
    %disp(strcat("i is", num2str(i)))
    for j = (i+1):length(torque_mat(:,1)) 
       % disp(strcat("j is", num2str(j)))
       torque = torque_mat(i,:) + torque_mat(j,:);
       force = force_mat(i,:) + force_mat(j,:);
       
       if nnz(torque) == 1 && nnz(force) == 0 %check which combos give you a torque in one direction and no force
            torque_result_mat_edge = [torque_result_mat_edge; [i j torque]];
       end
       
       if nnz(torque) == 0 && nnz(force) == 1 %check which combos give you a force in one direction and no torque
            force_result_mat_edge = [force_result_mat_edge; [i j force]];
       end
       
    end
end

force_result_mat_edge
torque_result_mat_edge