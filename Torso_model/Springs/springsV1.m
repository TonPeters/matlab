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
m_1 = 7;
m_2 = 7;
m_3 = 7;



% clear parameters
clearvars -except g l_1 l_2 l_3 l_F1 l_F2 th_2_min th_2_max th_0_min th_0_max g K_leg K_trunk L0_leg L0_trunk FR_leg FR_trunk
plotsettings

%% Settings
n = 6;     % grid size
m_4 = 20;       % kg, mass of the arms
M_arms = 52;    % Nm, load of the arms
N_leg = 2;
N_trunk = 2;

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
param = [6.704 5.672 0.207 -0.00756 0.0095];
P1 = param(1);
P2 = param(2);    
Kls_1 = param(3);
Kd_1 = param(4);
Kc_1 = param(5);
param = [2.548 0.0290 0.0097];
P3 = param(1);
Kc_2 = param(2);
Kd_2 = param(3);
q2_gravity_offset = 2/180*pi;   % offset angle of gravity of joint q2

%% create variable set
% joint velocities
dq_ = [1; -1];
qdd = 0.33;

% joint angles
q0_ = linspace(th_0_min,th_0_max,n);
q2_ = linspace(th_2_min,th_2_max,n);
[q0,q2,dq] = meshgrid(q0_,q2_,dq_);

q1 = angle0_to_angle1(q0);

% other parameters depenant on joint angles
th_F1 = spindle1_to_Fangle1(angle0_to_spindle1(q0)); % angle of force applied by spindle 1
th_F2 = spindle2_to_Fangle2(angle2_to_spindle2(q2)); % angle of force applied by spindle 2
dq1_dq0 = ratio_dq1_q0(q0);                          % change of q1 dependant on q0

syms qs0 qs2
%% Compute system model
% gravity contribution
g_leg = g.*(cos(q0).*(P1+l_1*m_4)+...             
    (dq1_dq0-1).*cos(q1-q0).*(P2+l_2*m_4)+...     
    (1-dq1_dq0).*cos(q2-q1+q0+q2_gravity_offset).*(P3+l_3*m_4));
g_trunk = g*(P3+m_4.*l_3).*cos(q2-q1+q0+q2_gravity_offset);

% payload of the arms
g_arms_trunk = M_arms;
g_arms_leg = M_arms*(1-dq1_dq0);

% spring contribution
lspr1 = angle0_to_spring1(q0);                                 % length spring leg
dlspr1_dq = sym_partial_derivative(angle0_to_spring1(qs0),qs0);
dlspr1_dq_num = double(subs(dlspr1_dq,qs0,q0));
gs_leg = -N_leg*K_leg.*(L0_leg-lspr1).*dlspr1_dq_num;                        % force gas spring leg
dlspr2_dq = sym_partial_derivative(angle2_to_spring2(qs2),qs2);
dlspr2_dq_num = double(subs(dlspr2_dq,qs2,q2));
lspr2 = angle2_to_spring2(q2);                                 % length spring trunk
gs_trunk = -N_trunk*K_trunk.*(L0_trunk-lspr2).*dlspr2_dq_num;                % force gas spring trunk

% potential contribution
G_leg = g_leg+gs_leg+g_arms_leg;
G_trunk = g_trunk+gs_trunk+g_arms_trunk;

% negative velocity extra spindle force
thFgs1 = spring1_to_Fangle1(lspr1);
F_gs1 = -N_leg.*FR_leg*sin(thFgs1);
thFgs2 = spring2_to_Fangle2(lspr2);
F_gs2 = -N_trunk.*FR_trunk*sin(thFgs2);

G_leg(:,:,2) = G_leg(:,:,2)+F_gs1(:,:,2);
G_trunk(:,:,2) = G_trunk(:,:,2)+F_gs2(:,:,2);

% inertia contribution by arms
load Inertia_arms_20kg.mat
D_m4_leg = D_m4*[qdd;0];
D_arms_leg = double(subs(D_m4_leg(1),{'q0','q2'},{q0,q2}));
D_arms_leg(:,:,2) = -D_arms_leg(:,:,2);
D_m4_trunk = D_m4*[0;qdd];
D_arms_trunk = double(subs(D_m4_trunk(2),{'q0','q2'},{q0,q2}));
D_arms_trunk(:,:,2) = -D_arms_trunk(:,:,2);

% inertia contribution by torso mechanism
% D_torso = [0,0;0,0];
D_m123_leg = D_torso*[qdd;0];
D_leg = double(subs(D_m123_leg(1),{'q0','q2'},{q0,q2}));
D_leg(:,:,2) = -D_leg(:,:,2);
D_m123_trunk = D_torso*[0;qdd];
D_trunk = double(subs(D_m123_trunk(2),{'q0','q2'},{q0,q2}));
D_trunk(:,:,2) = -D_trunk(:,:,2);

% force transfer
% S11 = 1.*l_F1.*sin(th_F1).*(r_sp*r_gear1);
% S22 = 1.*l_F2.*sin(th_F2).*(r_sp*r_gear2);
% S = [S11,0;0,S22];
% invS = inv(S);

% motor torques by gravity
tau_G_leg = G_leg./(l_F1.*sin(th_F1).*(r_sp*r_gear1));
tau_G_trunk = G_trunk./(l_F2.*sin(th_F2).*(r_sp*r_gear2));

% friction
tau_fric_pr_leg = tau_G_leg.*sign(dq.*tau_G_leg)*Kls_1;
tau_fric_leg = tau_fric_pr_leg+sign(dq).*Kc_1+Kd_1;
tau_fric_trunk = sign(dq).*Kc_2+Kd_2;

% motor torques
tau_leg = (tau_fric_leg+tau_G_leg+D_arms_leg+D_leg);             %.*(r_sp*r_gear1);
tau_trunk = (tau_fric_trunk+tau_G_trunk+D_arms_trunk+D_trunk);       %.*(r_sp*r_gear1);


%% show results 2D
% motor limits
tau_lim = [tau_nom_max,-tau_nom_max;tau_nom_max,-tau_nom_max];
tau_lim2_leg = [1500/r_sp/r_gear1,-1500/r_sp/r_gear1;1500/r_sp/r_gear1,-1500/r_sp/r_gear1];
tau_lim2_trunk = [1500/r_sp/r_gear2,-1500/r_sp/r_gear2;1500/r_sp/r_gear2,-1500/r_sp/r_gear2];
th_0_lim = [th_0_min;th_0_max]./pi*180;
th_2_lim = [th_2_min;th_2_max]./pi*180;

fig1 = figure; scr r;
ax1(1) = subplot(2,2,1);
plot(th_0_lim,tau_lim,'r--'); hold all;
plot(squeeze(q0(:,:,1)).'./pi*180,squeeze(tau_leg(:,:,1)).'); hold all; 
ylabel('Torque (+ vel) [Nm]');
plot(th_0_lim,tau_lim2_leg,'k--');
ax1(2) = subplot(2,2,3);
plot(th_0_lim,tau_lim,'r--'); hold all;
plot(squeeze(q0(:,:,2)).'./pi*180,squeeze(tau_leg(:,:,2)).');hold all; 
ylabel('Torque (- vel) [Nm]');
plot(th_0_lim,tau_lim2_leg,'k--');
xlabel('angle joint 1 [deg]');
ax1(3) = subplot(2,2,2);
plot(th_2_lim,tau_lim,'r--'); hold all;
plot(squeeze(q2(:,:,1))./pi*180,squeeze(tau_trunk(:,:,1)));hold all; 
plot(th_2_lim,tau_lim2_trunk,'k--');
ax1(4) = subplot(2,2,4);
plot(th_2_lim,tau_lim,'r--'); hold all;
plot(squeeze(q2(:,:,1))./pi*180,squeeze(tau_trunk(:,:,2)));hold all; 
plot(th_2_lim,tau_lim2_trunk,'k--');
xlabel('angle joint 2 [deg]');
axes(ax1(1));
linkaxes(ax1(1:2),'x')
xlim(th_0_lim);
axes(ax1(3));
linkaxes(ax1(3:4),'x')
xlim(th_2_lim);


linkaxes(ax1([1 3]),'y')
linkaxes(ax1([2 4]),'y')
set(ax1([3 4]),'yticklabel',[])
setplot
leg_list = {'lim','lim','q_{min}'};
for i=2:n-1
    leg_list{i+2} = '';
end
leg_list{n+2} = 'q_{max}';
legend_outside(fig1,leg_list)


all_grids_on();
set([ax1(1) ax1(3)],'xticklabel',[]);
% save_report(fig1,'','Torque_N22','customleg',[10 6]);
%% 3D plot
% define colormap
cmax1 = max(max(max(max(abs(tau_leg)))));
cmin1 = min(min(min(min(abs(tau_leg)))));
cmax2 = max(max(max(max(abs(tau_trunk)))));
cmin2 = min(min(min(min(abs(tau_trunk)))));
cm_length = 60;
CV_1 = fix((abs(tau_leg)-cmin1)./(cmax1-cmin1).*cm_length)+1;
CV_2 = fix((abs(tau_trunk)-cmin2)./(cmax2-cmin2).*cm_length)+1;

figure; scr lt;
ax_1(1) = subplot(2,1,1);
surf(squeeze(q0(:,:,1))./pi*180,squeeze(q2(:,:,1))./pi*180,squeeze(tau_leg(:,:,1)),squeeze(CV_1(:,:,1)),...
    'CDataMapping','direct');
title('positive velocity');
xlabel('q_0 [deg]'); ylabel('q_2 [deg]'); zlabel('Current 1 (+vel)');
ax_1(2) = subplot(2,1,2);
surf(squeeze(q0(:,:,2))./pi*180,squeeze(q2(:,:,2))./pi*180,squeeze(tau_leg(:,:,2)),squeeze(CV_1(:,:,2)),...
    'CDataMapping','direct');
xlabel('q_0 [deg]'); ylabel('q_2 [deg]'); zlabel('Current 1 (-vel)');
col_leg = colorbar;
caxis([0 cm_length]);
xtick = get(col_leg,'Ytick');
Xlabel = {};
for i=1:1:length(xtick)
    Xlabel{i} = num2str(xtick(i)./cm_length.*(cmax1-cmin1)+cmin1,2);
end
set(col_leg,'YTickLabel',Xlabel)



figure; scr rt;
ax_2(1) = subplot(2,1,1);
surf(squeeze(q0(:,:,1))./pi*180,squeeze(q2(:,:,1))./pi*180,squeeze(tau_trunk(:,:,1)),squeeze(CV_2(:,:,1)),...
    'CDataMapping','direct');
xlabel('q_0 [deg]'); ylabel('q_2 [deg]'); zlabel('Current 2 (+vel)');
ax_2(2) = subplot(2,1,2);
surf(squeeze(q0(:,:,2))./pi*180,squeeze(q2(:,:,2))./pi*180,squeeze(tau_trunk(:,:,2)),squeeze(CV_2(:,:,2)),...
    'CDataMapping','direct');
xlabel('q_0 [deg]'); ylabel('q_2 [deg]'); zlabel('Current 2 (-vel)');
col_trunk = colorbar;
caxis([0 cm_length]);
xtick = get(col_trunk,'Ytick');
Xlabel = {};
for i=1:1:length(xtick)
    Xlabel{i} = num2str(xtick(i)./cm_length.*(cmax2-cmin2)+cmin2,2);
end
set(col_trunk,'YTickLabel',Xlabel)

hlink1 = linkprop(ax_1,{'CameraPosition','CameraUpVector'});
hlink2 = linkprop(ax_2,{'CameraPosition','CameraUpVector'});


%% invest current needed
tau_1max = max(max(max((tau_leg))));
tau_2max = max(max(max((tau_trunk))));
tau_1min = min(min(min((tau_leg))));
tau_2min = min(min(min((tau_trunk))));
range1 = tau_1max-tau_1min;
range2 = tau_2max-tau_2min;

format_str = 'Motor %1.0i, min current = %4.2f, max current = %4.2f, range = %4.2f \n';
fprintf(format_str,1,tau_1min,tau_1max,range1)
fprintf(format_str,2,tau_2min,tau_2max,range2)

%% motor characteristics
%   http://www.maxonmotor.nl/maxon/view/product/motor/dcmotor/re/re35/323890
%





























all_grids_on();
