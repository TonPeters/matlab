clear all; close all; clc;

%% get measures from model
plotsettings
run torso_measures_NX

th_2_min = spindle2_to_angle2(min_spindle2);    % min angle q2
th_2_max = spindle2_to_angle2(max_spindle2);    % max angle q2
th_0_min = spindle1_to_angle0(min_spindle1);    % min angle q0
% th_0_max = spindle1_to_angle0(max_spindle1);    % max angle q0
th_0_min= 0.05;
min_spindle1 = angle0_to_spindle1(th_0_min);

% motor parameters
K_m     = 29.2e-3;      % Nm/A,         Motor torque constant
K_elm   = 10;           % A/V_input,    Gain from input Voltage to Current     
r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
l_ls    = 0.002;        % m,           Lead of the spindle
r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation
I_m     = 79.2e-7;      % kg.m2,        Motor rotor inertia
l_s = 0.25;             % m,            Estimate ?????? lenght of the rotating screw
I_ls = 10e-6.*l_s;      % kg.m2,        Spindle screw inertia
tau_nom_max = 101e-3;   % Nm            Maximum nominal motor torque

enc_counts = 500*4;
% speed settings
le_range = max_spindle1-min_spindle1;
le_speed = le_range/3;

%% trunk motor
le_motor_rad = le_speed*r_sp*r_gear1;
le_motor_rpm = le_motor_rad/2/pi*60;
desired_gear_rpm = (7000/60*2*pi)/(le_speed*r_sp);
% disp(['Current nominal rpm = ',num2str(tr_motor_rpm)]);

tau_nom_max_gear = tau_nom_max*r_gear1;
tau_nom_max_gear_desired = 1700/r_sp;
desired_gear_torque = tau_nom_max_gear_desired/tau_nom_max;

le_force_max = tau_nom_max*r_gear1*r_sp;
disp(['Desired gear by torque is ',num2str(desired_gear_torque),' and by nom speed is ',num2str(desired_gear_rpm)])
disp(['Gear 2.5:1, nominal rpm = ',num2str(le_motor_rpm),' with maximum force = ',num2str(le_force_max)]);

enc2m = 2*pi/enc_counts*r_gear1/r_sp;
res4 = enc2m*1000;
enc_per_mm = 1/enc2m/1000;
%% new gearing 8.7:1
r_gear1_new = r_gear1*7/2;

% nominal speed
le_motor_rad = le_speed*r_sp*r_gear1_new;
le_motor_rpm = le_motor_rad/2/pi*60;


le_force_max = tau_nom_max*r_gear1_new*r_sp;
le_tau_gear_new = le_force_max./r_sp;
disp(['New Gear 8.7:1, nominal rpm = ',num2str(le_motor_rpm),' with maximum force = ',num2str(le_force_max)]);

% reduce speed
le_speed_red = le_range/5;
le_motor_rad_red = le_speed_red*r_sp*r_gear1_new;
le_motor_rpm = le_motor_rad_red/2/pi*60;
disp(['New Gear 8.7:1, speed reduced to 5 sec on range, nominal rpm = ',num2str(le_motor_rpm),' with maximum force = ',num2str(le_force_max)]);

enc2m = 2*pi/enc_counts*r_gear1_new/r_sp;
res6 = enc2m*1000;
enc_per_mm = 1/enc2m/1000;
%% new gearing 10.8:1
r_gear1_new = r_gear1*r_gear2;

% nominal speed
le_motor_rad = le_speed*r_sp*r_gear1_new;
le_motor_rpm = le_motor_rad/2/pi*60;


le_force_max = tau_nom_max*r_gear1_new*r_sp;
le_tau_gear_new = le_force_max./r_sp;
disp(['New Gear 10.8:1, nominal rpm = ',num2str(le_motor_rpm),' with maximum force = ',num2str(le_force_max)]);

% reduce speed
le_speed_red = le_range/5;
le_motor_rad_red = le_speed_red*r_sp*r_gear1_new;
le_motor_rpm = le_motor_rad_red/2/pi*60;
disp(['New Gear 10.8:1, speed reduced to 5 sec on range, nominal rpm = ',num2str(le_motor_rpm),' with maximum force = ',num2str(le_force_max)]);

%% resolution reduction
enc2m = 2*pi/enc_counts*r_gear1_new/r_sp;
res12 = enc2m*1000;
enc_per_mm = 1/enc2m/1000;

disp(['Resolution 2.5 -> ',num2str(res4),' mm/count, 8.7 -> ',num2str(res6),' mm/count, 10.8 -> ',num2str(res12),' mm/count']);