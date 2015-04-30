clear all; 
close all; 
clc;
%-------------------------------------------------------------------------%
% Model trunk, place the legs in most upright position to get full gravity
% change in the range.
%  
% Check the different contributions in the motor torque required
%   - Kinematic energy trunk 
%   - Potential energy trunk
%   - Kinematic energy motor
%-------------------------------------------------------------------------%

%% get measures from model
run torso_measures_NX

% temporary parameters
lAG = di(A,G);
lAE = di(A,E);
lAC = di(A,C);
lGJ = di(G,J);
lJK = di(J,K);
lJH = di(J,H);
lDJ = di(D,J);
lJL = di(J,L);
aBAZ = an(B,A,A+[0;-1;0]);

% parameters
g = 9.81;                                       % gravitation
th_2_min = spindle2_to_angle2(min_spindle2);    % min angle q2
th_2_max = spindle2_to_angle2(max_spindle2);    % max angle q2
th_0_min = spindle1_to_angle0(min_spindle1);    % min angle q0
th_0_max = spindle1_to_angle0(max_spindle1);    % max angle q0
l_F2 = lJK;                                      % arm of applied spindle force on joint 2
l_F1 = lAE;                                      % arm of applied spindle force on joint 1
l_1 = lAG;
l_2 = lGJ;
l_3 = lJL;

% estimates
l_cm1 = l_1/2;
l_cm2 = l_2/2;
l_cm3 = l_3/2;
m_1 = 3.5;
m_2 = 5;
m_3 = 10;



% clear parameters
clearvars -except g l_1 l_2 l_3 l_F1 l_F2 th_2_min th_2_max th_0_min th_0_max g m_3 l_cm3
plotsettings

%% Settings
load model_tau_NSprings0_0_m3.5_5_10_20.mat
q0_r = th_0_min;

%% system parameters
% known parameters
K_m     = 29.2e-3;      % Nm/A,         Motor torque constant
K_elm   = 10;           % A/V_input,    Gain from input Voltage to Current     
r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
l_ls    = 0.002;        % m,           Lead of the spindle
r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation
I_m     = 79.2e-7;      % kg.m2,        Motor rotor inertia
I_g2    = 9.1e-7;       % kg.m2,        Gearbox trunk inertia
l_s = 0.25;            % m,            Estimate ?????? lenght of the rotating screw
I_ls = 10e-6.*l_s;      % kg.m2,        Spindle screw inertia
tau_nom_max = 101e-3;    % Nm            Maximum nominal motor torque

% handy parameters
r_VtoF1 = (K_elm*K_m*r_gear1*r_sp); % N/V,  translation factor from voltage to spindle force 1
r_VtoF2 = (K_elm*K_m*r_gear2*r_sp); % N/V,  translation factor from voltage to spindle force 2

% estimated parameters
P1 = 6.0848;                    % P1 = lcm1*m1 + l1*m2 + l1*m3
P2 = 5.0254;                    % P2 = lcm2*m2 + l2*m3
P3 = 2.4724;                    % P4 = lcm3*m3
q2_gravity_offset = 2/180*pi;   % offset angle of gravity of joint q2
K_c1 = 0.0736.*r_VtoF1;         % N,    Coulomb friction force 1
K_c2 = 0.0995.*r_VtoF2;         % N,    Coulomb friction force 2
K_offset2 = -0.033.*r_VtoF2;    % N,    Direction dependant coulomb friction force 2

%% Reference trajectory
% offset by angle of th0
qd0_r = 0;
qdd0_r = 0;

q1 = angle0_to_angle1(q0_r);
q_offset = q0_r-q1;

% reference trajectory
Ts = 0.1;                   % sample time dt
t_duration = 4;             % time of the trajectory
vel_max = pi/2/t_duration;           % max ref vel
acc_max = vel_max;          % max ref acc
jerk = 1000*acc_max;        % infinite jerk
[q2_r,qd2_r,qdd2_r,time] = reference_generator(th_2_min,th_2_max,Ts,vel_max,acc_max,jerk);

% show reference trajectory
h_ref = figure; scr rt;
subplot(3,1,1); plot(time,q2_r); ylabel('angle [rad]');
subplot(3,1,2); plot(time,qd2_r); ylabel('velocity [rad/s]');
subplot(3,1,3); plot(time,qdd2_r); ylabel('acceleration [rad/s2]');
linkaxes(get(h_ref,'children'),'x');

%% torques and forces
% compute joint torque needed
disp('start D');
T_joint = double((subs(tau_D(2),{'q0','q2','qdd0','qdd2'},{q0_r.',q2_r.',qdd0_r.',qdd2_r.'}))).';
disp('end D');

% compute motor torque needed
disp('start D drive');
T_drive = double((subs(tau_D_drive(2),{'q0','q2','qd0','qd2','qdd0','qdd2'},{q0_r.',q2_r.',qd0_r.',qd2_r.',qdd0_r.',qdd2_r.'}))).';
disp('end D drive');

T_tot = T_joint+T_drive;

%% show results

figure; scr r;
subplot(1,1,1);
plot(time,T_joint,time,T_drive,time,T_tot); hold all;
% plot(t,ones(size(t)).*tau_nom_max,'r--');
legend('joint','drive','tot');

all_grids_on();