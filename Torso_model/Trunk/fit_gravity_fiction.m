clear all; 
% close all; 
clc;
%-------------------------------------------------------------------------%
% Model: Trunk
%-------------------------------------------------------------------------%

%% get measures from model
run torso_measures_NX.m

% m1 = M_LRL+M_LFL;
% m2 = M_UL;
% m3 = 20;        % mass of the trunk
lAG = di(A,G);
lAE = di(A,E);
lAC = di(A,C);
lGJ = di(G,J);
lJK = di(J,K);
lJH = di(J,H);
lDJ = di(D,J);
lJL = di(J,L);
aBAZ = an(B,A,A+[0;-1;0]);
th_leg = 0.56;
g = 9.81;

th_0_min = spindle2_to_angle2(min_spindle2);
th_0_max = spindle2_to_angle2(max_spindle2);
l1 = lJL;
lcm1 = l1*0.7;
lF = lJK;
th_r = -th_leg+angle0_to_angle1(th_leg);
m1 = 8;
m2 = 0;
m1_lcm1 = 2.63;

% clear parameters
clearvars -except l1 lcm1 lF th_r m1 m2 th_0_min th_0_max g m1_lcm1
plotsettings
%% load data to fit
load '/home/ton/git_ton/matlab/Torso_Identification/add_mass/upper_3mass.mat'
q_meas = th_0k./180*pi+th_r; qF0 = spindle2_to_Fangle2(angle2_to_spindle2(q_meas));
tau_meas = u_0k;

% cut data
q_min = 1.1;
q_max = 2.2;
indices = find(q_meas>q_min & q_meas<q_max);
q_m = q_meas(indices);
tau_m = tau_meas(indices);
% figure; plot(q_m,tau_m,'.');


% compute velocity
tau_bound = 0.05;
indices2 = find(tau_m<tau_bound);
qd_m = ones(size(q_m));
qd_m(indices2) = -1;
% figure; plot(qd_m,tau_m,'.')

%% optimization fit
% initial gues
param0 = [m1_lcm1;100;100];

[param,fval,exitflag] = fminunc(@(param)costfunc_friction_gravity(param,q_m,qd_m,tau_m),param0)

m1_lcm1 = param(1);
Kc = param(2);
Kd = param(3);

% m1_lcm1 = 2.63;
% Kc = 0;
% Kd = 0;




%% compare fit to measurement
% Parameters
l_ls = 0.002;
rsp = 2*pi/l_ls;
rgear = 13/3;
Kmm = 29.2e-3;             % Nm/A,     torque constant
Kelm = 10;

% V_mod = ((g*(m1_lcm1+m2*l1)*cos(q_m-th_r))./lF./sin(thF) + Kc*sign(qd_m)+Kd)./rsp/rgear/Kmm/Kelm;

% Moment needed
M_joint = (g*(m1_lcm1+m2*l1)*cos(q_m-th_r));

% Effective force needed
F_eff = M_joint./lF;

% applied force needed
thF = -0.9902.*q_m+3.152;     % Angle force applied
F_appl = F_eff./sin(thF);
    
% add Friction force
Ffr = Kc*sign(qd_m)+Kd;
F_appl_fric = F_appl+Ffr;

% Spindle torque needed
T_sp = F_appl_fric/rsp;

% Motor torque needed
T_m = T_sp/rgear;

% Motor current needed
I_m = T_m/Kmm;

% Input Voltage needed
V_mod = I_m/Kelm;

figure;
plot(q_m,tau_m,q_m,V_mod);

