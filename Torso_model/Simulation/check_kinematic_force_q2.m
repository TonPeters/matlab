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
m_4 = 20;     % mass of the arms


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
q0_r = th_0_min;
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

% reference trajectory spindle
x_sp2 = angle2_to_spindle2(q2_r);
q_sp2 = x_sp2*r_sp; 
diff_sp_q2 = gradient(q_sp2)./gradient(q2_r);       % dsp/dq0 numerical
indi = find(gradient(q2_r)==0);                     % remove nan 
diff_sp_q2(indi) = diff_sp_q2(indi-1);
qd_sp2 = qd2_r.*diff_sp_q2;                         % qd spindle0
ddiff_sp_q2 = gradient(qd_sp2)./gradient(qd2_r);
indi = find(isnan(ddiff_sp_q2));
ddiff_sp_q2(indi) = zeros(size(indi));
qdd_sp2 = qdd2_r.*ddiff_sp_q2;
% q_sp2 = q_sp2-q_sp2(1);

diff2_sp_q = gradient(diff_sp_q2)./gradient(q2_r);
indi = find(gradient(q2_r)==0);
diff2_sp_q(indi) = zeros(size(indi));
qdd_sp2 = qdd2_r.*diff_sp_q2+diff2_sp_q.*qd2_r.^2;
figure; scr lt;
plot(time,qdd_sp2,time,qdd_sp2);

% reference trajectory motor
q_m2 = q_sp2*r_gear2;
qd_m2 = qd_sp2.*r_gear2;
qdd_m2 = qdd_sp2.*r_gear2;

% show reference trajectory
h_ref = figure; scr rt;
subplot(3,1,1); plot(time,[q2_r,q_sp2./300,q_m2./1000]); ylabel('angle [rad]');
subplot(3,1,2); plot(time,[qd2_r,qd_sp2./300,qd_m2./1000]); ylabel('velocity [rad/s]');
subplot(3,1,3); plot(time,[qdd2_r,qdd_sp2./300,qdd_m2./1000]); ylabel('acceleration [rad/s2]');
linkaxes(get(h_ref,'children'),'x');

%% torques and forces

% Torque transfer ratios
thF2 = spindle2_to_Fangle2(angle2_to_spindle2(q2_r));     % angle of force applied by spindle 2
tau_qtom = 1./l_F2./sin(thF2)./r_sp./r_gear2;                  % transfer from joint torque to motor torque

% compute joint torque needed
load model_tau_NSprings0_0_m3.5_5_10_20.mat
disp('start D');
T_q_m2 = double((subs(tau_D(2),{'q0','q2','qdd0','qdd2'},{q0_r.',q2_r.',qdd0_r.',qdd2_r.'}))).';
disp('end D');

% compute motor torque needed
disp('start D drive');
T_drive = double((subs(tau_D_drive(2),{'q0','q2','qd0','qd2','qdd0','qdd2'},{q0_r.',q2_r.',qd0_r.',qd2_r.',qdd0_r.',qdd2_r.'}))).';
disp('end D drive');
%%

% compute joint torque needed
T_q = (m_3*l_cm3^2+m_4*l_3^2).*qdd2_r;
V_q = g*(m_3*l_cm3+l_3*m_4).*cos(q2_r+q_offset);
% V_q = P3*g.*cos(q2+q_offset);

% lead screw rotatation inertia
T_ls = I_ls*qdd_sp2;

% compute motor torque needed
T_q_m = T_q.*tau_qtom;
V_q_m = V_q.*tau_qtom;
T_ls_m = T_ls./r_gear2;
T_m_m = I_m*qdd_m2;
T_g2_m = I_g2*qdd_m2;

% Totatl motor torque
tau = T_q_m+ V_q_m + T_ls_m + T_m_m;

I_drive = T_ls_m+T_m_m+T_g2_m;
I_joint = T_q_m;
I_tot = I_drive+I_joint;

figure; scr r;
subplot(3,1,1);
plot(time,T_q,time,V_q);
legend('T','V');
subplot(3,1,2);
plot(time,I_joint,time,I_drive,time,I_tot,time,T_q_m2,'--',time,T_drive); hold all;
% plot(t,ones(size(t)).*tau_nom_max,'r--');
legend('joint','drive','tot');
subplot(3,1,3);
plot(time,T_ls_m,time,T_m_m,time,T_g2_m); hold all;
legend('ls','m','g');


all_grids_on();