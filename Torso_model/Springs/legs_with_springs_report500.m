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
% N_leg = 0;          % number of springs in the leg
F1_leg = 500;       % load of springs in the leg
Motor_limit = 1500;

% variables
q0 = linspace(th_0_min,th_0_max,n_0);  % 
q2 = linspace(th_2_min,th_2_max,n_2);
qd = [1,        -1];  % only sign of velocity is used
M_4 = [0,52];  % Nm, load of the arms
N_leg = [0,2];          % number of springs in the leg
qdd = [0.32;0];         % trajectory, 1s acc, 2s vel, 1s dec, that covers the full range (qdd = [0.32;0.41];)


% zero shoulders
q0c = linspace(th_0_min,th_0_max,n_0);
q2c = shoulders_zero(q0c);

%% limits
% motor
F_lim_leg = [tau_nom_max,-tau_nom_max;tau_nom_max,-tau_nom_max].*(r_sp*r_gear1);
F_lim2_leg = [1/r_sp/r_gear1,-1/r_sp/r_gear1;1/r_sp/r_gear1,-1/r_sp/r_gear1].*Motor_limit*(r_sp*r_gear1);
% angle limits
th_0_lim = [th_0_min;th_0_max]./pi*180;
th_2_lim = [th_2_min;th_2_max]./pi*180;

%% simulate
% legend list q0
leg_q2 = {};

col = [-3 0];
fig2 = figure; scr rt;
col2 = [-5 -5; -0 -2];
for ii = 1:2
    for i=1:2
        for j=1:2

            %% compute kinematic limits
            for k=1:length(q2)
                %% Run simulation
                F_leg = sim_leg2(q0,q2(k),qd(i),qdd,m_4,M_4(j),N_leg(ii),F1_leg);

                %% show results 2D
                % plot results
                figure(fig2);
                ax((i-1)*2+j) = subplot(2,2,(i-1)*2+j);
                % plot max
                if ii==1
                    ph_q2(k,(i-1)*2+j) = plot(q0./pi*180,F_leg,'color',ps.list_div_XL{end,end-5}); hold all;
                else
                    ph_q2(k,(i-1)*2+j) = plot(q0./pi*180,F_leg,'color',ps.list_div_XL{1,end-k*3+2}); hold all;
                end
                leg_q2{k} = ['$q_2=',num2str(q0(k)),'$'];

            end

            %% Compute shoulders centered

            M_4c = 0;
            qdc = -1;
            F_legc = sim_leg2(q0c,q2c,qd(i),qdd,m_4,M_4(j),N_leg(ii),F1_leg);
            if ii==1
                ph_c_g((i-1)*2+j) = plot(q0c./pi*180,F_legc,'color',ps.list_div_XL{end,end-5}); hold all;
            else 
                ph_c((i-1)*2+j) = plot(q0c./pi*180,F_legc,'color',ps.list_div_XL{2,7}); hold all;
            end

            % motor limits
            ph_l((i-1)*2+j,:) = plot(th_0_lim,F_lim_leg,'--','color',ps.list_div_XL{3,7}); 
        end

    end
end

%% plot limits and make up
for ii = 1:4, uistack(ph_l(ii,:),'bottom'); end;
% plot(th_0_lim,F_lim2_leg,'k--');
% title('Trunk');
linkaxes(ax,'xy');
xlim(th_0_lim)
ylim([-2100 3000]);
axes(ax(1));
ylabel('Force [N] (+)');
tit(1) =title('Moment 0 Nm');
axes(ax(2));
tit(2) = title('Moment 52 Nm');
axes(ax(4));
xlabel('angle ankle joint [deg]');
axes(ax(3));
ylabel('Force [N] (-)');
xlabel('angle ankle joint [deg]');

set(ax([2,4]),'yticklabel',[]);
set(ax([2,1]),'xticklabel',[]);
all_grids_on();

% leg_q0{length(q0)+1} = 'center';
leg_list = {'$q_2$ min','$q_2$ max','center','motor limits','no springs'};
leg = legend_outside(fig2,leg_list,[ph_q2(:,2);ph_c(2);ph_l(2,1);ph_c_g(2)]);

%% save_figure
fig_save = false;
if fig_save
    % legend size
    set(leg,'units','centimeters','interpreter','latex');
    leg_pos = get(leg,'position');
    leg_width = leg_pos(3);

    % scale figure
    m1 = 0.5; m2 = 1.0; m3 = 0.3; m4 = 1.5; m5 = 0.3+leg_width; m6 = 0.3;
    scale = 0.7;
    plot_size = [10 6].*scale;
    figsize = plot_size.*[1,1]+[m4+m5+m6 m1+m2+m3];
    setplot(fig2,figsize,{m1,m2,m3,m4,m5,m6},9);


    % set(tit,'position',[2.5,3.1,0])

    set(fig2,'PaperUnits', 'centimeters');
    set(fig2,'Papersize',figsize);
    set(fig2,'PaperPositionMode','manual');
    set(fig2,'PaperPosition',[0 0 figsize]);

    dir_file = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/leg_springs500';
    print(fig2,'-dpdf',dir_file)
end
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

all_grids_on();
