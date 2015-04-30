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
m_4 = 0;


% clear parameters
clearvars -except g l_1 l_2 l_3 l_F1 l_F2 th_2_min th_2_max th_0_min th_0_max g m_1 m_2 m_3 m_4 l_cm1 l_cm2 l_cm3
plotsettings

%% Load data
load('data_trunk.mat');
measurements = [1,2,3,4,5];
sets = [1,2];
q2_est = linspace(th_2_min,th_2_max,40).';


fig1 = figure; scr rt;
fig2 = figure; scr lt;
for meas = 1:3
    
    % show data
    figure(fig2); 
    plot(sdata{meas}.q2_s{1},(sdata{meas}.tau_f{1}+sdata{meas}.tau_f{2})./2,'color',ps.list_div{meas,2}); hold all;
    plot(sdata{meas}.q2_s{1},(sdata{meas}.tau_f{1}-sdata{meas}.tau_f{2})./2,'color',ps.list_div{meas,2}); hold all;
    
    for set = sets
        figure(fig1);
        plot(sdata{meas}.q2_m{set},sdata{meas}.tau_m{set},'color',ps.list_div{meas,1}); hold all;
%         plot(sdata{meas}.q2_s{set},sdata{meas}.tau_f{set},'--','color',ps.list_div{meas,2});
    end
    
    % calculate fit
    param = [2.548, 0.0290, 0.0097];
    
    % model calculation
    tau_est_p{meas} = cost_trunk_fricCD_mass(param,q2_est,zeros(size(q2_est)),sdata{meas}.m2,sdata{meas}.q_offset,1);
    tau_est_n{meas} = cost_trunk_fricCD_mass(param,q2_est,zeros(size(q2_est)),sdata{meas}.m2,sdata{meas}.q_offset,-1);

    figure(fig1);
    plot(q2_est,tau_est_p{meas},'color',ps.list_div{meas,2});
    plot(q2_est,tau_est_n{meas},'color',ps.list_div{meas,2});
end
figure(fig1); 
ylim([-0.06 0.12]);
xlim([1.05 2.35]);
% tau_mass = -cost_legs_fricLS([0,0,0],q0_est,zeros(size(q0_est)),sdata{meas}.q2,0.004);
% plot(q0_est,tau_mass,'color',ps.tuedarkblue);

% check params
param


all_grids_on();