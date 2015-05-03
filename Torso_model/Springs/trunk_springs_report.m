clear all; 
close all; 
clc;
%-------------------------------------------------------------------------%
% Model: Trunk
%-------------------------------------------------------------------------%
set(0,'defaulttextinterpreter','latex')
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

th_2_lim = [th_2_min;th_2_max]./pi*180;
%% Settings
n_0 = 2;              % grid size
n_2 = 10;              % grid size
m_4 = 20;           % kg, mass of the arms
% M_4 = 52;           % Nm, load of the arms
N_trunk = 2;          % number of springs in the leg
% F1_trunk = 800;       % load of springs in the leg

% variables
qd = [1,        -1];  % only sign of velocity is used
q2 = linspace(th_2_min,th_2_max,n_2);
F1_trunk = [800];

% plot settings
leg_springs = {};
fig2 = figure; scr rt;
col = {ps.tueblue,ps.tuegreen};
for i =1:length(F1_trunk)
    for j=1:1:2
        %% effect of spring 
        % spring load
        B_trunk  = 404e-3+2*30e-3;  % complete extended length m
        A_trunk  = 175e-3;          % spring motion range m
        % F1_trunk = 800;             % force min length
        X_trunk  = 1.35;            % f2/f1
        F2_trunk = F1_trunk(i)*X_trunk;% force min length
        K_trunk  = (F2_trunk-F1_trunk(i))/(A_trunk-10e-3); % spring constant (slope N/m)
        FR_trunk = 60;              % direction dependant force
        L0_trunk = B_trunk  + (F1_trunk(i))/K_trunk; % length at zero energy m (not reachable)

        % length spring
        lspr2 = angle2_to_spring2(q2);
        % force spring
        F_spring = -2*K_trunk.*(L0_trunk-lspr2);

        if qd(j)<0
            F_spring = F_spring-N_trunk*FR_trunk;
        end
        
        % angle of spring on joint
        th_spring = spring2_to_Fangle2(lspr2);
        % moment on joint
        M_spring  = F_spring.*sin(th_spring).*l_Fgs2;
        % length lead screw
        lls2 = angle2_to_spindle2(q2);
        % angle on lead screw
        th_ls = spindle2_to_Fangle2(lls2);
        % Force on lead screw
        F_ls = M_spring./l_F2./sin(th_ls);
        
        figure(fig2);
%         ax(j) = subplot(2,1,j);
        pl(i,j) = plot(q2./pi*180,F_ls,'color',col{j}); hold all

    end
end

%% plot limits and make up
% plot(th_0_lim,F_lim2_leg,'k--');
% title('Trunk');
% linkaxes(ax,'xy');
xlim(th_2_lim)
% ylim([-1500 3300]);
% axes(ax(1));
% ylabel('Force [N] (+)');
% axes(ax(2));
ylabel('Force [N]');
xlabel('angle hip joint [deg]');
leg = legend('$+$ vel','$-$ vel','location','northwest');
set(leg,'interpreter','latex');
% set(ax([1]),'xticklabel',[]);
all_grids_on();

dir_file = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/';
filename = 'trunk_springs';
save_report(fig2,dir_file,filename,'custom',[7 4]);


%% plot effect


% effect = 
F1_effect = 1.*sin(th_spring).*l_Fgs2./l_F2./sin(th_ls);
figure;
plot(q2,F1_effect)


all_grids_on();
