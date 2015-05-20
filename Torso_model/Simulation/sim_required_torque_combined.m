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
m_1 = 5;
m_2 = 5;
m_3 = 5;



% clear parameters
clearvars -except th_2_min th_2_max th_0_min th_0_max
plotsettings

%% Simulation settings


%% Transmission parameters
% known parameters
K_m     = 29.2e-3;      % Nm/A,         Motor torque constant
K_elm   = 10;           % A/V_input,    Gain from input Voltage to Current     
r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
l_ls    = 0.002;        % m,           Lead of the spindle
r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation
I_m     = 79.2e-7;      % kg.m2,        Motor rotor inertia
l_s = 0.25;            % m,            Estimate ?????? lenght of the rotating screw
I_ls = 10e-6.*l_s;   % kg.m2,        Spindle screw inertia
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

% reference trajectory legs
Ts = 0.1;                   % sample time dt
t_duration = 4;             % time of the trajectory
% vel_max = pi/2/t_duration;  % max ref vel
vel_max = (th_0_max-th_0_min)/3;  % max ref vel
acc_max = vel_max;          % max ref acc
jerk = 1000*acc_max;        % infinite jerk
[q0_r,qd0_r,qdd0_r,time0] = reference_generator(th_0_min,th_0_max,Ts,vel_max,acc_max,jerk);

q1_r = angle0_to_angle1(q0_r);
qd1_r = angle0_to_angle1(qd0_r);
qdd1_r = angle0_to_angle1(qdd0_r);

% reference trajectory trunk
th_2_end = th_2_max;
Ts = 0.1;                   % sample time dt
t_duration = 4;             % time of the trajectory
% vel_max = pi/2/t_duration;  % max ref vel
vel_max = (th_2_end-th_2_min)/3;  % max ref vel
acc_max = vel_max;          % max ref acc
jerk = 1000*acc_max;        % infinite jerk
[q2_r,qd2_r,qdd2_r,time2] = reference_generator(th_2_min,th_2_end,Ts,vel_max,acc_max,jerk);

time = time2;
% % reference trajectory spindle and motor
% x_sp0 = angle0_to_spindle1(q0_r);
% q_sp0 = x_sp0*r_sp; q_sp0 = q_sp0-q_sp0(1);
% qd_sp0 = [diff(q_sp0)./Ts;0];
% qdd_sp0 = [diff(qd_sp0)./Ts;0];
% q_m0 = q_sp0*r_gear1;
% qd_m0 = [diff(q_m0)./Ts;0];
% qdd_m0 = [diff(qd_m0)./Ts;0];

% show reference trajectory
h_ref = figure; scr rt;
subplot(3,1,1); plot(time,[q0_r,q2_r]); ylabel('angle [rad]');
subplot(3,1,2); plot(time,[qd0_r,qd2_r]); ylabel('velocity [rad/s]');
subplot(3,1,3); plot(time,[qdd0_r,qdd2_r]); ylabel('acceleration [rad/s2]');
linkaxes(get(h_ref,'children'),'x');


%% Simulation
load model_tau_NSprings0_0_m3.5_5_10_0final.mat

% model
clc; disp('start tau_D');
tau_D_subs = (subs(tau_D,{'q0','q2','qdd0','qdd2'},{q0_r.',q2_r.',qdd0_r.',qdd2_r.'}));
tau_D_num = zeros(size(tau_D_subs));
for i=1:1:length(tau_D_num)
    clc; i
    tau_D_num(1,i) = double(tau_D_subs(1,i));
end
for j=1:1:length(tau_D_num)
    clc; j
    tau_D_num(2,j) = double(tau_D_subs(2,j));
end

clc; disp('start tau_G');
tau_G_num = double(subs(tau_G,{'q0','q2'},{q0_r.',q2_r.'}));

% substitute one by one
clc; disp('start tau_C');
tau_C_subs = (subs(tau_C,{'q0','q2','qd0','qd2'},{q0_r.',q2_r.',qd0_r.',qd2_r.'}));
tau_C_num = zeros(size(tau_C_subs));
for i=1:1:length(tau_C_num)
    clc; i
    tau_C_num(1,i) = double(tau_C_subs(1,i));
end
for j=1:1:length(tau_C_num)
    clc; j
    tau_C_num(2,j) = double(tau_C_subs(2,j));
end

clc; disp('start Tau');
Tau = tau_C_num+tau_D_num+tau_G_num;


% tau_C_num1 = (subs(tau_C,{'q2','qd2','qdd2'},{0,0,0}));
% tau_D_num1 = (subs(tau_D,{'q2','qd2','qdd2'},{0,0,0}));
% tau_G_num1 = (subs(tau_G,{'q2','qd2','qdd2'},{0,0,0}));
% 
% tau_C_num2 = (subs(tau_C_num1,{'q0','qd0'},{q0_r.',qd0_r.'}));
% % substitute one by one
% tau_C_num = zeros(size(tau_C_num2));
% for i=1:1:length(tau_C_num)
%     clc; i
%     tau_C_num(1,i) = double(tau_C_num2(1,i));
% end
% for j=1:1:length(tau_C_num)
%     clc; j
%     tau_C_num(2,j) = double(tau_C_num2(2,j));
% end
% 
% % tau_C_num = (subs(tau_C_num1,{'q0','qd0'},{q0_r.',qd0_r.'}));
% clc; disp('start tau_D');
% tau_D_num = double(subs(tau_D_num1,{'q0','qd0','qdd0'},{q0_r.',qd0_r.',qdd0_r.'}));
% clc; disp('start tau_G');
% tau_G_num = double(subs(tau_G_num1,{'q0','qd0','qdd0'},{q0_r.',qd0_r.',qdd0_r.'}));
% clc; disp('start Tau');
% Tau = tau_D_num+tau_G_num;

%% motor dynamics
% I_ls_m = I_ls/r_gear1^2;    % translation lead screw inertia to motor space
% I_dr = I_ls_m+I_m;          % total drive train inertia
% Tau_m = I_dr.*qdd_m0;      % Torque required to overcome motor inertia

%% show results
figure
subplot(3,1,1);
plot(time,qdd0_r,time,qdd2_r); ylabel('qdd ref [rad/s2]');
legend('0','2');
subplot(3,1,2);
plot(time,Tau(1,:),time,tau_C_num(1,:),time,tau_D_num(1,:),time,tau_G_num(1,:)); hold all;
% plot(time,tau_nom_max.*ones(size(time)));
ylabel('Torque 1 [Nm]');
subplot(3,1,3);
plot(time,Tau(2,:),time,tau_C_num(2,:),time,tau_D_num(2,:),time,tau_G_num(2,:)); hold all;
% plot(time,tau_nom_max.*ones(size(time)));
ylabel('Torque 2 [Nm]');xlabel('Time [s]');
legend({'tot','C','D','G'});
linkaxes(get(gcf,'children'),'x');
all_grids_on();



