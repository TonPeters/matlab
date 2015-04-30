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
N_leg = 0;
N_trunk = 0;

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
I_g2    = 9.1e-7;       % kg.m2,        Gearbox trunk inertia
l_s = 0.25;             % m,            Estimate ?????? lenght of the rotating screw
I_ls = 10e-6.*l_s;      % kg.m2,        Spindle screw inertia
tau_nom_max = 101e-3;   % Nm            Maximum nominal motor torque

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

%% Variables
syms q0 qd0 qdd0 q2 qd2 qdd2 F1 F2
q1 = angle0_to_angle1(q0);
qd1 = sym_time_derivative(q1,q0,qd0);
qdd1 = sym_time_derivative(qd1,[q0;qd0],[qd0;qdd0]);

l_sp1 = angle0_to_spindle1(q0);
th_F1 = spindle1_to_Fangle1(l_sp1);
l_sp2 = angle2_to_spindle2(q2);
th_F2 = spindle2_to_Fangle2(l_sp2);

tau = [F1;F2];
q = [q0;q2];
qd = [qd0;qd2];
qdd = [qdd0;qdd2];

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

%% motor inertia
% spindle position
q_sp = [angle0_to_spindle1(q0);angle2_to_spindle2(q2)];

% motor positions and derivatives
q_m = q_sp.*[r_sp*r_gear1;r_sp*r_gear2];
qd_m = sym_time_derivative(q_m,q,qd);
qdd_m = sym_time_derivative(qd_m,[q;qd],[qd;qdd]);

% spindle inertia in motor frame
I_ls1_m = I_ls./r_gear1;
I_ls2_m = I_ls./r_gear2;

D_drive = [I_ls1_m+I_m+I_g2 0; 0 I_ls2_m+I_m+I_g2];

tau_D_drive = D_drive*qdd_m;

%% equations of motion D(q)qdd +C(q,qd)+G(q) = S(q) tau, (q is joint, tau is motor torque)
T_qd = sym_partial_derivative(T,qd);

% mass matrix
D = sym_partial_derivative(T_qd.',qd);

% centripedal and coriolis terms C
C_dt_T_qd = sym_time_derivative(T_qd.',q,qd);
C_T_q = -sym_partial_derivative(T,q).';
C = C_dt_T_qd+C_T_q;

% potential terms
G_g = sym_partial_derivative(V_g,q).';
G = G_g;

% input matrix
S = sym_partial_derivative(Qnc,tau);
% convert to motor torque
S(1:2,1) = S(1:2,1).*r_sp.*r_gear1;
S(1:2,2) = S(1:2,2).*r_sp.*r_gear2;

%% Contributions of terms on the required input torque
Sinv = inv(S);

tau_D = Sinv*D*qdd;
tau_C = Sinv*C;
tau_G = Sinv*G;
Tau = tau_D+tau_C+tau_G;

filename = ['model_tau_NSprings',num2str(N_leg),'_',num2str(N_trunk),'_m',num2str(m_1),'_',num2str(m_2),'_',num2str(m_3),'_',num2str(m_4s)];
save(['../Simulation/',filename,'.mat'],'tau_D','tau_C','tau_G','Tau','D','Sinv','tau_D_drive','D_drive');





