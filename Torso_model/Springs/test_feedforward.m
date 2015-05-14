clear all; 
close all; 
clc;
%-------------------------------------------------------------------------%
% Model: Trunk
%-------------------------------------------------------------------------%

%% get measures from model
run torso_measures_NX

% temporary parameters
lAG = di(A,G);
lAE = di(A,E);
lAC = di(A,C);
lAF = di(A,F);
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
l_Fgs2 = lJK;                                      % arm of applied spring force on joint 2
l_Fgs1 = lAF;                                      % arm of applied spring force on joint 1
l_1 = lAG;
l_2 = lGJ;
l_3 = lJL;

% estimates
l_cm1 = l_1/2;
l_cm2 = l_2/2;
l_cm3 = l_3/2;
m_1 = 7;
m_2 = 7;
m_3 = 7;



% clear parameters
clearvars -except g l_1 l_2 l_3 l_F1 l_F2 th_2_min th_2_max th_0_min th_0_max g K_leg K_trunk L0_leg L0_trunk FR_leg FR_trunk l_Fgs2 l_Fgs1
plotsettings

th_0_min = 0.05;        % absolute minimum of th_0

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

 

%% Settings
n_0 = 10;              % grid size
n_2 = 2;              % grid size
m_4 = 20;           % kg, mass of the arms
% M_4 = 52;           % Nm, load of the arms
N_leg = 2;          % number of springs in the leg
F1_leg = 275;       % load of springs in the leg
Motor_limit = 1700;

% variables
q0 = linspace(th_0_min,th_0_max,n_0);  % 
q2 = [th_2_max, th_2_min];
qd = [1 -1];  % only sign of velocity is used
M_4 =[0 52];  % Nm, load of the arms
qdd = [0.32;0];         % trajectory, 1s acc, 2s vel, 1s dec, that covers the full range (qdd = [0.32;0.41];)


% zero shoulders
q0c = linspace(th_0_min,th_0_max,n_0);
q2c = shoulders_zero(q0c);
% qdc = 1;
% M_4c = 0;

%% limits
% motor
F_lim_leg = [tau_nom_max,-tau_nom_max;tau_nom_max,-tau_nom_max].*(r_sp*r_gear1);
F_lim2_leg = [1/r_sp/r_gear1,-1/r_sp/r_gear1;1/r_sp/r_gear1,-1/r_sp/r_gear1].*Motor_limit*(r_sp*r_gear1);
% angle limits
th_0_lim = [th_0_min;th_0_max]./pi*180;
th_2_lim = [th_2_min;th_2_max]./pi*180;

%% simulate
format_str = 'lsp1 = %5.4f, lsp2 = %5.4f, tau1 = %5.4f, tau2 = %5.4f\n';
% q0 = [th_0_min, th_0_min,th_0_max,th_0_max];
% q2 = [th_2_min, th_2_max,th_2_min,th_2_max];
% qd = [1, 1, 1, 1;...
%     1, 1, 1, 1];
q0 = [th_0_max];
q2 = [th_2_min];
qd = [1;...
    1];
% kinematic limits
for i=1:length(q0);
    F_leg = sim_leg2(q0(i),q2(i),qd(1,i),0,20,0,0,1)./(r_sp*r_gear1);    
    F_trunk = sim_trunk2(q0(i),q2(i),qd(2,i),0,20,0,0,1)./(r_sp*r_gear2);
    
    lsp1 = angle0_to_spindle1(q0(i));
    lsp2 = angle2_to_spindle2(q2(i));
    fprintf(format_str,lsp1,lsp2,F_leg,F_trunk)
    
end

%% cpp
m_arms = 0;
angles = [q0,angle0_to_angle1(q0),q2];
% angleForce = [spindle1_to_Fangle1(lsp1), spindle2_to_Fangle2(lsp2)]
P1 = 6.58;
    P2 = 4.32;
    P3 = 2.47;
    Kmu1 = 0.288;
    Kc1 = 0.0075;
    Kc2 = 0.029;
    Kd2 = 0.0097;
C1 = 0.150052;
    C2 = -0.115090;
    C3 = 0.614223;
    C4 = 0.174737;
    C5 = -0.064849;
    C6 = 0.205912;
    C7 = 0.171033;
    C8 = 0.107369;
    C9 = 0.756629;
    C10 = -0.133132;
    C11 = -0.779956;
    C12 = 0.132722;
    C13 = -0.155586;
    C14 = 2.913677;
    C24 = 0.162258;
    C25 = -0.157983;
    C26 = -0.096279;
    C27 = -0.701898;
    C28 = 1075.352975;
C29 = 2756.347553;
    g    = 9.81;
    gravity_offset = 0.0349;
    l1 = 0.389978;
    l2 = 0.411498;
    l3 = 0.469972;
%     params_q0q1[4] = 11.68;
%     params_q0q1[3] = -24.33;
%     params_q0q1[2] = 20.18;
%     params_q0q1[1] = -7.847;
    params_q0q1 = [2.957;-7.847;20.18;-24.33;11.68];
%     dq1_dq0 = params_q0q1(1)+params_q0q1(2)*q0+params_q0q1(3)*q0^2+params_q0q1(4)*q0^3+params_q0q1(5)*q0^4
% forces1 = g*(P3+l3*m_arms)*cos(angles(3)+angles(1)-angles(2)+...
%     gravity_offset)/(sin(angleForce(2))*C28)
% forces0 = g*(cos(angles(1))*(P1+l1*m_arms)+(dq1_dq0-1)*cos(angles(2)-angles(1))*(P2+l2*m_arms)+(1-dq1_dq0)*cos(angles(3)-angles(2)+angles(1)+gravity_offset)*(P3+l3*m_arms))/(sin(angleForce(1))*C29)
