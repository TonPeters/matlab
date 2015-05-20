clear all; 
% close all; 
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

th_0_min = 0.1;        % absolute minimum of th_0

% motor parameters
K_m     = 29.2e-3;      % Nm/A,         Motor torque constant
K_elm   = 10;           % A/V_input,    Gain from input Voltage to Current     
r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
l_ls    = 0.002;        % m,            Lead of the spindle
r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation
I_m     = 79.2e-7;      % kg.m2,        Motor rotor inertia
l_s = 0.25;             % m,            Estimate ?????? lenght of the rotating screw
I_ls = 10e-6.*l_s;      % kg.m2,        Spindle screw inertia
tau_nom_max = 101e-3;   % Nm            Maximum nominal motor torque

%% Settings
n_0 = 10;              % grid size
n_2 = 10;              % grid size
m_4 = 20;           % kg, mass of the arms
% M_4 = 52;           % Nm, load of the arms
N_trunk = 2;          % number of springs in the leg
F1_trunk = 675;       % load of springs in the leg
Motor_limit = 1600;

q0 = [th_0_max, th_0_min];  % 
qd = [1,        -1];  % only sign of velocity is used
M_4 = [52,       0];  % Nm, load of the arms

% joint velocities and accelerations
qdd = [0;0.43];         % trajectory, 1s acc, 2s vel, 1s dec, that covers the full range (qdd = [0.31;0.43];)

% joint angles
q2 = linspace(th_2_min,th_2_max,n_2);


fig2 = figure; scr rt;
%% compute kinematic limits
for i=1:length(q0)
    %% Run simulation
    F_trunk = sim_trunk2(q0(i),q2,qd(i),qdd,m_4,M_4(i),N_trunk,F1_trunk);

    %% show results 2D
    % motor limits
    F_lim_trunk = [tau_nom_max,-tau_nom_max;tau_nom_max,-tau_nom_max].*(r_sp*r_gear2);
    F_lim2_trunk = [1/r_sp/r_gear2,-1/r_sp/r_gear2;1/r_sp/r_gear2,-1/r_sp/r_gear2].*Motor_limit*(r_sp*r_gear2);
    % angle limits
    th_0_lim = [th_0_min;th_0_max]./pi*180;
    th_2_lim = [th_2_min;th_2_max]./pi*180;

    % plot results
    figure(fig2);
    % plot max
    plot(q2./pi*180,F_trunk,'color',ps.tueblue); hold all;

end

%% Compute shoulders centered
q0c = linspace(th_0_min,th_0_max,n_0);
q2c = shoulders_zero(q0c);
M_4c = 0;
qdc = -1;
F_trunkc = sim_trunk2(q0c,q2c,qdc,qdd,m_4,M_4c,N_trunk,F1_trunk);
plot(q2c./pi*180,F_trunkc,'color',ps.tuegreen); hold all;


% plot limits and make up
plot(th_2_lim,F_lim_trunk,'r--'); 
plot(th_2_lim,F_lim2_trunk,'k--');
title('Trunk');
ylabel('Force [N]');
xlabel('angle hip joint [deg]');
all_grids_on();
    
%% plot all results
plot_all = false;
if plot_all
    % plot results
    fig1 = figure; scr l;
    ax1(1) = subplot(2,1,1);
    plot(squeeze(q0(:,:,1)).'./pi*180,squeeze(F_leg(:,:,1)).'); hold all;
    ax1(2) = subplot(2,1,2);
    plot(squeeze(q0(:,:,2)).'./pi*180,squeeze(F_leg(:,:,2)).');hold all; 

    % plot limits and make up
    axes(ax1(1));
    plot(th_0_lim,F_lim_leg,'r--'); 
    plot(th_0_lim,F_lim2_leg,'k--');
    ylabel('Force (+ vel) [N]'); title('Legs');
    axes(ax1(2));
    plot(th_0_lim,F_lim_leg,'r--'); 
    ylabel('Force (- vel) [N]');
    plot(th_0_lim,F_lim2_leg,'k--');
    xlabel('angle joint 1 [deg]');

    % axes settings
    linkaxes(ax1(1:2),'x')
    xlim(th_0_lim);
    all_grids_on();
    set(ax1(1),'xticklabel',[]);
end

%% effect of spring 
% % spring load
% B_trunk  = 404e-3+2*30e-3;  % complete extended length m
% A_trunk  = 175e-3;          % spring motion range m
% % F1_trunk = 800;             % force min length
% X_trunk  = 1.35;            % f2/f1
% F2_trunk = F1_trunk*X_trunk;% force min length
% K_trunk  = (F2_trunk-F1_trunk)/(A_trunk-10e-3); % spring constant (slope N/m)
% FR_trunk = 60;              % direction dependant force
% L0_trunk = B_trunk  + (F1_trunk)/K_trunk; % length at zero energy m (not reachable)
% 
% % length spring
% lspr2 = angle2_to_spring2(q2);
% % force spring
% F_spring = -2*K_trunk.*(L0_trunk-lspr2);
% % angle of spring on joint
% th_spring = spring2_to_Fangle2(lspr2);
% % moment on joint
% M_spring  = F_spring.*sin(th_spring).*l_Fgs2;
% % length lead screw
% lls2 = angle2_to_spindle2(q2);
% % angle on lead screw
% th_ls = spindle2_to_Fangle2(lls2);
% % Force on lead screw
% F_ls = M_spring./l_F2./sin(th_ls);
% 
% % figure;
% plot(q2./pi*180,F_ls)

%% save results
% filename = ['Legs_N',num2str(N_leg),'_F',num2str(F1_leg),'_Ma',num2str(M_arms)];
% save_report(fig1,'/home/ton/Dropbox/Linux/Report/Images/Springs/',filename,'customleg',[8 8]);
% filename = ['Trunk_N',num2str(N_trunk),'_F',num2str(F1_trunk),'_Ma',num2str(M_arms)];
% save_report(fig2,'/home/ton/Dropbox/Linux/Report/Images/Springs/',filename,'customleg',[8 8]);

all_grids_on();
