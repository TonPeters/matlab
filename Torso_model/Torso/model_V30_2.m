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
%   Mass estimates Arne
m_1 = 3.5;
m_2 = 5;
m_3 = 10;
%   Mass equal devided
% m_1 = 5;
% m_2 = 5;
% m_3 = 5;
%   Mass estimates by experiments
% m_1 = 3.4;
% m_2 = 3.4;
% m_3 = 10.5;


% clear parameters
clearvars -except g l_1 l_2 l_3 l_F1 l_F2 th_2_min th_2_max th_0_min th_0_max g m_3 l_cm3 l_cm2 l_cm1 m_1 m_2 m_3 K_leg K_trunk L0_leg L0_trunk
plotsettings

%% Settings
% number of springs used
N_leg = 2;
N_trunk = 2;

% mass contribution arms
m_4 = 20;            % mass of the arms
m_4s = m_4;      	% mass in the shoulder
m_4a = m_4-m_4s;    % remaining mass goes in the arm
l_cm4 = 0.3;        % distance center of mass shoulder
q3 = 0;             % orientation of arm (q3=0 means on side of the trunk


%% system parameters
% known parameters
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

% handy parameters
r_VtoF1 = (K_elm*K_m*r_gear1*r_sp); % N/V,  translation factor from voltage to spindle force 1
r_VtoF2 = (K_elm*K_m*r_gear2*r_sp); % N/V,  translation factor from voltage to spindle force 2

% estimated parameters
% param = [6.704 5.672 0.207 -0.00756 0.0095];
% param = [7.1447  5.1355 0.2497   -0.0075    0.0077];
param = [6.5754    4.3237    0.2881  0.0  0.0075];
P1 = param(1);
P2 = param(2);    
Kls_1 = param(3);
Kd_1 = param(4);
Kc_1 = param(5);
% param = [2.548 0.0290 0.0097];
param = [2.47 0.0290 0.0097];
P3 = param(1);
Kc_2 = param(2);
Kd_2 = param(3);
q2_gravity_offset = 2/180*pi;   % offset angle of gravity of joint q2

% inertia estimates
Jm1 = 0.15;
Jm2 = 0.15;
Jm_sp = [Jm1, 0; 0, Jm2];

%% Variables
syms q0 qd0 qdd0 q2 qd2 qdd2 F1 F2
q1 = angle0_to_angle1(q0);
qd1 = sym_time_derivative(q1,q0,qd0);
qdd1 = sym_time_derivative(qd1,[q0;qd0],[qd0;qdd0]);
dq1_dq0 = sym_partial_derivative(q1,q0);

l_sp1 = angle0_to_spindle1(q0);
th_F1 = spindle1_to_Fangle1(l_sp1);
l_sp2 = angle2_to_spindle2(q2);
th_F2 = spindle2_to_Fangle2(l_sp2);

tau = [F1;F2];
q = [q0;q2];
qd = [qd0;qd2];
qdd = [qdd0;qdd2];

% motor speeds
q_m1 = r_gear1*r_sp*angle0_to_spindle1(q0);
q_m2 = r_gear2*r_sp*angle2_to_spindle2(q2);
ratio_qdd_m1 = sym_partial_derivative(q_m1,q0);
qdd_m1 = ratio_qdd_m1*q0;
ratio_qdd_m2 = sym_partial_derivative(q_m2,q2);
qdd_m2 = ratio_qdd_m2*q2;
%% joint positions described in world frame
xq1 = l_1*cos(q0);
yq1 = l_1*sin(q0);
xq2 = xq1-l_2*cos(q1-q0);
yq2 = yq1+l_2*sin(q1-q0);
xq3 = xq2+l_3*cos(q2-q1+q0);
yq3 = yq2+l_3*sin(q2-q1+q0);

xqd1 = -l_1*sin(q0)*qd0;
yqd1 = l_1*cos(q0)*qd0;
xqd2 = xqd1+l_2*sin(q1-q0)*(qd1-qd0);
yqd2 = yqd1+l_2*cos(q1-q0)*(qd1-qd0);
xqd3 = xqd2-l_3*sin(q2-q1+q0)*(qd2-qd1+qd0);
yqd3 = yqd2+l_3*cos(q2-q1+q0)*(qd2-qd1+qd0);

x1 = l_cm1*cos(q0);
y1 = l_cm1*sin(q0);
x2 = xq1-l_cm2*cos(q1-q0);
y2 = yq1+l_cm2*sin(q1-q0);
x3 = xq2+l_cm3*cos(q2-q1+q0);
y3 = yq2+l_cm3*sin(q2-q1+q0);
x4 = xq3+l_cm4*cos(q2-q1+q0+q3-pi/2);
y4 = yq3+l_cm4*sin(q2-q1+q0+q3-pi/2);

xd1 = -l_cm1*sin(q0)*qd0;
yd1 = l_cm1*cos(q0)*qd0;
xd2 = xqd1+l_cm2*sin(q1-q0)*(qd1-qd0);
yd2 = yqd1+l_cm2*cos(q1-q0)*(qd1-qd0);
xd3 = xqd2-l_cm3*sin(q2-q1+q0)*(qd2-qd1+qd0);
yd3 = yqd2+l_cm3*cos(q2-q1+q0)*(qd2-qd1+qd0);
xd4 = xqd3-l_cm4*sin(q2-q1+q0+q3-pi/2)*(qd2-qd1+qd0);
yd4 = yqd3+l_cm4*cos(q2-q1+q0+q3-pi/2)*(qd2-qd1+qd0);

%% Lagrange equations d/dt T,qd -T,q +V,q = Qnc^T
% potential energy by gravity
V_m1 = g*m_1*l_cm1*sin(q0);
V_m2 = g*m_2*(l_1*sin(q0)+l_cm2*sin(-q0+q1));
V_m3 = g*m_3*(l_1*sin(q0)+l_2*sin(-q0+q1)+l_cm3*sin(q0-q1+q2));
V_m4s = g*m_4s*(l_1*sin(q0)+l_2*sin(-q0+q1)+l_3*sin(q0-q1+q2));
V_m4a = g*m_4a*(l_1*sin(q0)+l_2*sin(-q0+q1)+l_3*sin(q0-q1+q2)+l_cm4*sin(q0-q1+q2+q3-pi/2));
V_g = V_m1+V_m2+V_m3+V_m4s+V_m4a;

% potential energy by gas springs
lspr1 = angle0_to_spring1(q0);                                 % length spring leg
V_spring1 = 0.5*N_leg*K_leg*(L0_leg-lspr1)^2;                   % Spring energy leg
lspr2 = angle2_to_spring2(q2);                                 % length spring trunk
V_spring2 = 0.5*N_trunk*K_trunk*(L0_trunk-lspr2)^2;             % Spring energy trunk
V_gs = V_spring1+V_spring2;


V = V_g+V_gs;

% kinetic energy
T_m1 = 1/2*m_1*(xd1^2+yd1^2);
T_m2 = 1/2*m_2*(xd2^2+yd2^2);
T_m3 = 1/2*m_3*(xd3^2+yd3^2);
T_m4s = 1/2*m_4s*(xqd3^2+yqd3^2);
T_m4a = 1/2*m_4a*(xd4^2+yd4^2);
T = T_m1+T_m2+T_m3+T_m4s+T_m4a;

% non conservative forces
Qnc1 = l_F1*sin(th_F1)*F1;
Qnc2 = l_F2*sin(th_F2)*F2;
Qnc = [Qnc1;Qnc2];

%% equations of motion D(q)qdd +C(q,qd)+G(q) = S(q) tau
% gravity contribution
g_leg = g.*(cos(q0).*(P1+l_1*m_4)+...             
    (dq1_dq0-1).*cos(q1-q0).*(P2+l_2*m_4)+...     
    (1-dq1_dq0).*cos(q2-q1+q0+q2_gravity_offset).*(P3+l_3*m_4));
g_trunk = g*(P3+m_4.*l_3).*cos(q2-q1+q0+q2_gravity_offset);
G_g = [g_leg;g_trunk];

% spring contribution
lspr1 = angle0_to_spring1(q0);                                 % length spring leg
gs_leg = -N_leg*K_leg*(L0_leg-lspr1)^2;                        % force gas spring leg
lspr2 = angle2_to_spring2(q2);                                 % length spring trunk
gs_trunk = -N_trunk*K_trunk*(L0_trunk-lspr2)^2;                % force gas spring trunk
G_gs = [gs_leg;gs_trunk];

% potential contribution
G = G_g-G_gs; 

% inertia contribution by arms
Tm4_qd = sym_partial_derivative(T_m4s+T_m4a,qd);
D_m4 = sym_partial_derivative(Tm4_qd.',qd);

% inertia contribution by torso mechanism
D_torso = [0,0;0,0];
T_torso_qd = sym_partial_derivative(T_m1+T_m2+T_m3,qd);
D_torso = sym_partial_derivative(T_torso_qd.',qd);

% motor dynamics
I_ls_m = [I_ls/r_gear1^2, 0; 0, I_ls/r_gear2^2];    % translation lead screw inertia to motor space
I_dr = I_ls_m+[I_m,0;0,I_m];                        % total drive train inertia
D_dr = I_dr*[ratio_qdd_m1, 0; 0, ratio_qdd_m2];     % total drive train inertia in joint frame
Tau_m = D_dr*qdd;                                   % Torque required to overcome motor inertia

% force transfer
S11 = 1.*l_F1.*sin(th_F1).*(r_sp*r_gear1);
S22 = 1.*l_F2.*sin(th_F2).*(r_sp*r_gear2);
S = [S11,0;0,S22];
invS = inv(S);

%% motor inertia
% spindle position
q_sp = [angle0_to_spindle1(q(1));angle2_to_spindle2(q(2))];

% motor positions and derivatives
q_m = q_sp;
% qd_m = sym_time_derivative(q_m,q,qd);
% qdd_m = sym_time_derivative(qd_m,[q;qd],[qd;qdd]);

% spindle inertia in motor frame
% I_ls1_m = I_ls./r_gear1;
% I_ls2_m = I_ls./r_gear2;

% I_drive = [I_ls1_m+I_m+I_g2 0; 0 I_ls2_m+I_m+I_g2];

qd_first = sym_partial_derivative(q_m,q);
qd_sec = sym_partial_derivative(qd_first*qd,q);

C_drive = Jm_sp*qd_sec;
D_drive = Jm_sp*qd_first;

%% transfer to motor torque and add the friction components
Dm = invS*(D_m4)+D_drive;
Gm = invS*(G);

% friction components
Fprop = [sign(qd0)*Kls_1;...
        0               ].*Gm;
Ffric = [sign(qd0).*Kc_1+Kd_1;...
        sign(qd2).*Kc_2+Kd_2];
    
tau_m = Dm*qdd+Gm+Fprop+Ffric;

%% Contributions of terms on the required input torque

filename = ['model_new_ident_NSprings',num2str(N_leg),'_',num2str(N_trunk),'_m',num2str(m_1),'_',num2str(m_2),'_',num2str(m_3),'_',num2str(m_4s)];
save(['../Springs/',filename,'.mat'],'Dm','Gm','Fprop','Ffric','tau_m');

D_m4 = invS*D_m4;
% D_torso = invS*D_torso+D_dr;
D_torso = D_drive;
save('../Springs/Inertia_arms_20kg.mat','D_m4','D_torso','D_drive');




