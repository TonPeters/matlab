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
clearvars -except g l_1 l_2 l_3 l_F1 l_F2 th_2_min th_2_max th_0_min th_0_max g
plotsettings

%% Settings
m4 = 0;     % mass of the arms
n = 8;     % grid size

fig1 = figure;
s_row = 3;
s_col = 1;

%% plot data as reference
load_list = {'10','14','16'};
meas = 1;
joint = 1;
Kelm = 10;
Kmm = 29.2e-3;
q2_list = [1.01, 1.4, 1.6];
load(['/home/ton/git_ton/matlab/Legs_Identification/data_friction/m0kg_vel004_trunk',load_list{meas},'.mat']);
q0_meas = spindle1_to_angle0(enc(:,joint));
qd0_meas = ones(size(q0_meas));
qd0_meas(indices.n1) = -ones(length(indices.n1),1);
qd0_meas(indices.n2) = -ones(length(indices.n2),1);
tau_meas = u(:,1);


%% data to fit
%     q0_m = [q0_meas(indices.p1);q0_meas(indices.p2);q0_meas(indices.n1);q0_meas(indices.n2)];
%     qd0_m = [qd0_meas(indices.p1);qd0_meas(indices.p2);qd0_meas(indices.n1);qd0_meas(indices.n2)];
%     tau_m = [tau_meas(indices.p1);tau_meas(indices.p2);tau_meas(indices.n1);tau_meas(indices.n2)];
%     indices_list = {indices.p1, indices.p2, indices.n1, indices.n2};
indices_list = {indices.p1, indices.n1, indices.p2, indices.n2};
indices_s = indices.p1;
for k=1:2 %length(indices_list)
    indices = indices_list{k};

%         indi = find(q0_meas(indices)<57/180*pi);
    q0_m = [q0_meas(indices)];
    qd0_m = [qd0_meas(indices)];
    tau_m = [tau_meas(indices).*Kelm*Kmm];
    time_m = time(indices);

    % plot data to fit
    figure(fig1);
    subplot(s_row,s_col,1);
    plot(q0_m,tau_m,'color',ps.list_div{meas,1},'linewidth',0.5); hold all; 
end
        
        
        

%% system parameters
% known parameters
K_m     = 29.2e-3;      % Nm/A,         Motor torque constant
K_elm   = 10;           % A/V_input,    Gain from input Voltage to Current     
r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
l_ls    = 0.002;        % mm,           Lead of the spindle
r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation

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




%% create variable set
% joint velocities
dq0 = 1;
dq2 = 1;
dq = [1; -1];

% joint angles
q0 = linspace(th_0_min,th_0_max,n);
q2 = q2_list(meas);

q1 = angle0_to_angle1(q0);



% other parameters depenant on joint angles
thF1 = spindle1_to_Fangle1(angle0_to_spindle1(q0)); % angle of force applied by spindle 1
thF2 = spindle2_to_Fangle2(angle2_to_spindle2(q2)); % angle of force applied by spindle 2
dq1_q0 = ratio_dq1_q0(q0);                          % change of q1 dependant on q0


%% Compute system model
% Joint Torque legs
T_leg = g.*(cos(q0).*(P1+l_1*m4)+...             
    (dq1_q0-1).*cos(q1-q0).*(P2+l_2*m4)+...     
    (1-dq1_q0).*cos(q2-q1+q0+q2_gravity_offset).*(P3+l_3*m4));

tau_leg = 1/(l_F1*r_gear1*r_sp)./sin(thF1).*T_leg;

%% nonlinear functions
% applied force angle
C1 = 1/(l_F1*r_gear1*r_sp);
f1 = 1./sin(thF1);

% angle q0
C2 = g*(P1+l_1*m4);
f2 = cos(q0);

% angle q1
C3 = g*(P2+l_2*m4);
f3 = (dq1_q0-1).*cos(q1-q0);

% angle q2
C4 = g*(P3+l_3*m4);
f4 = (1-dq1_q0).*cos(q2-q1+q0+q2_gravity_offset);

%% results
figure(fig1);
subplot(s_row,s_col,1);
plot(q0,tau_leg);
ylabel('motor torque [Nm]');
subplot(s_row,s_col,2);
plot(q0,tau_leg./(f1.*C1),q0,f2.*C2,q0,f3.*C3,q0,f4.*C4)
ylabel('joint torque [Nm]');
subplot(s_row,s_col,3);
plot(q0,f2,q0,f3,q0,f4)
ylabel('f nonlin [-]');
linkaxes(get(gcf,'children'),'x');
all_grids_on();

%% effect of q2
fig2 = figure; scr rt; 

for i=1:3
%     (1-dq1_q0).*
    f4_i = cos(q2_list(i)-q1+q0+q2_gravity_offset);

    plot(q0,f4_i); hold all;
end