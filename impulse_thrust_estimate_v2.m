%% Part 1: Single case, no tolerances introduced into test stand

% Mass properties of the satellite

len = .304; %this is a foot in m
r = (len/2)*.95; %Assume thrusters are firing only 5% of the radius away from the outside

%Mass of components on outside (lbs) - assume these make a thin rectangular
%frame

m_frame = 1.6;
m_internal_struct = 1;
m_reg_2 = .5;
m_plumb_man = .75;
m_valves = .6;
m_avio = .5;
m_cg_adjust = .5;
m_outside = (m_frame + m_internal_struct + m_avio + m_reg_2 + m_plumb_man + m_valves + m_cg_adjust)*.454;
I_outside = (5/18)*m_outside*len^2;

%Mass of components on inside (lbs) - assume these components make a sphere
m_tank = 1.9; %Includes paintball reg
m_air = .35; %Conservative assumption - full tank of air
m_inside = (m_tank + m_air)*.454;
r_inside = 4*.0254; %Assume sphere with radius 4 inches
I_inside = (2/5)*m_inside*r_inside^2;

I_sc = I_inside + I_outside
m_sc = m_inside + m_outside
%Mass properties of test stand
 %assume two concentric rectangular plates .5" thick in all dimensions.
A_stand = 14^2 - 12^2;
V_stand = A_stand*1.25;
m_stand = .098*V_stand*.454 %multiply by density of 6061 Al in lb/in^3, then convert to kg
len_outside = 14*.0254;
len_inside = 12*.0254;
I_stand = (1/6)*m_stand*(len_outside^2 + len_inside^2)

I_sys = I_stand + I_sc;
delta_w = pi; %10 maneuvers to 15 deg/s, 16 to 2 deg/s

impulse = I_sys*delta_w/r %doesn't account for grav torques

alpha = pi/12; %assume you want to get to 15 deg/s in one second
cg_offset = .001;

force_no_grav = (I_sys*alpha)/(2*r)
force_with_grav = (m_sc*cg_offset*9.81 + I_sys*alpha)/(2*r) 

%% Part 2: Monte Carlo where test stand tolerances are varied

%Figures out what the test stand CG accuracy is when tolerances are
%introduced

%For now, assume that both bearings have the same placement error so the
%structure isn't rotated

t = .5; %thickness of bar in inches
h = 1.25; %height of bar in inches
gap = .25; %distance between bars
length = [12+2*t; 12+4*t+gap]; %Might want to add the third bar if we figure out that CG control of test stand is good

test_stand_grav_torque_vec = [];
total_CG_vec = [];

for i = 1:100 %Monte Carlo trials
    test_stand_grav_torque =  0; %amount to measure total torque gravity produces from offset CG of test stand
    CG_num = 0;
    CG_den = 0;
    
    for ring = 1:2 %Iterate through every ring, change to 3 if add a third ring to this
        bearing_tol = .002*2*(rand()-.5); %Random # between -.002 and .002 for the bearing placement, positive tolerance means it moves up
        length_tol = .002*2*([rand()-.5 rand()-.5 rand()-.5 rand()-.5]); %Array of 4 #'s with same range as above for length tolerances
        thick_tol = .001*2*([rand()-.5 rand()-.5 rand()-.5 rand()-.5]);
        height_tol = .001*2*([rand()-.5 rand()-.5 rand()-.5 rand()-.5]);
        
      
        %For sake of time, assume average length of 2nd bar (right side) affects
        %top and bottom CG. Assume thickness doesn't affect CG of each bar.
     
        
        length_adjust = [0 -2*t 0 -2*t]; %Sides are shorter by twice the thickness
        length_vec = length(ring) + length_adjust + length_tol; %Length vector
        thick_vec = thick_tol + t;
        height_vec = height_tol + h;
        
        CG_vec = [((length_vec(2) - t)/2)-bearing_tol -bearing_tol (-(length_vec(2) - t)/2)-bearing_tol -bearing_tol];
        rho_al = .0975; %lb/in^3 
        mass_vec = rho_al*length_vec.*thick_vec.*height_vec;
        CG = .0254*dot(CG_vec, mass_vec)/sum(mass_vec); %convert to m
        mass = sum(mass_vec)*.454; %convert to kg
        test_stand_grav_torque =test_stand_grav_torque + CG*mass*9.81;
        
        CG_num = CG_num + .0254*dot(CG_vec, mass_vec);
        CG_den = CG_den + sum(mass_vec);
    end
    total_CG_vec = [total_CG_vec; CG_num/CG_den];
    test_stand_grav_torque_vec = [test_stand_grav_torque_vec test_stand_grav_torque];
end

test_stand_CG = max(abs(total_CG_vec));
figure (1)
scatter(1:100, test_stand_grav_torque_vec)
hold on
yline(9.81*m_sc*.001)
hold off
legend("Test Stand", "Satellite w/ CG offset of 1 mm")
xlabel("trial")
ylabel("Gravitational Torque from offset CG (N*m)")
title("Test Stand vs Satellite Gravitational Torques")

%% Part 3: Trade Study of do we want more mass on test stand

%Look at ratio of torque to rotate entire structure vs torque to counter CG
%from satellite + test stand, see if adding more mass helps
%Density of test stand hasn't mattered up until this point

sc_CG = .001; %choose low value but assume in same direction as test stand CG
impulse_vec = [];
force_ratio_vec = [];
force_with_grav_vec = [];

for density_stand = 0:.005:.3 %sweeping through a ranges of densities for test stand, lb/in^3
    m_stand = density_stand*V_stand*.454; %V_stand found earlier, needs to be slightly updated
    len_outside = 14*.0254;
    len_inside = 12*.0254;
    I_stand = (1/6)*m_stand*(len_outside^2 + len_inside^2);
    I_sys = I_stand + I_sc;
    
    %From earlier, delta_w = pi/12 (15 deg/s) and you want to accelerate to
    %that in one second
    
    impulse = I_sys*delta_w/r; %doesn't account for grav torques
    impulse_vec = [impulse_vec impulse];
    
    force_no_grav = (I_sys*alpha)/(2*r);
    force_with_grav = ((m_sc*cg_offset + m_stand*test_stand_CG)*9.81 + I_sys*alpha)/(2*r); %assume CG offset for sat = 1 mm
    force_with_grav_vec = [force_with_grav_vec; force_with_grav];
    force_ratio = force_with_grav/force_no_grav;
    force_ratio_vec = [force_ratio_vec force_ratio];
end

figure (2)
densities = 0:.005:.3;
scatter(densities, impulse_vec)
xline(.0975)
xlabel('test stand density (lb/in^3)')
ylabel('impulse (N*m)')
title('How impulse varies with Test Stand Mass')
legend('impulse', 'Al density')

figure (3)
scatter(densities, force_ratio_vec)
xline(.0975)
xlabel('test stand density (lb/in^3)')
ylabel('force with gravity/force without gravity (N*m)')
title('How magnitude gravitational effects vary with Test Stand Mass')
legend('force ratio', 'Al density')

figure (4)
scatter(densities, force_with_grav_vec)
xline(.0975)
xlabel('test stand density (lb/in^3)')
ylabel('force with gravity (N*m)')
title('How Force varies with Test Stand Mass when CGs are offset')
legend('force ', 'Al density')
