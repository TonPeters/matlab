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

%% Load data


load('data_legs.mat');
meas = 1;
set = 1;
i=1;

fig1 = figure; scr rt;
plot(sdata{meas}.q0_m{set},sdata{meas}.tau_m{set},'color',ps.list_div{i,1}); hold all;
plot(sdata{meas}.q0_s{set},sdata{meas}.tau_f{set},'color',ps.list_div{i,2});

%% optimization
param0 =    [10];
lb =        [-1000000];
ub =        [1000000];

% lsqnonlin
opts = optimset('lsqnonlin');
opts = optimset(opts,'tolfun',1E-16,'display','off','tolx',1E-16);
problem = createOptimProblem('lsqnonlin','objective',...
    @(param)cost_single_f14(param,sdata{meas}.q0_s{set},sdata{meas}.tau_f{set},sdata{meas}.q2),...
    'x0',param0,'lb',lb,'ub',ub,'options',opts);
ms = MultiStart('Display','off');
n_startpoints = 2;
[param,fval,exitflag] = run(ms,problem,n_startpoints);

if (exitflag ~= 1)
    display(['No solution, exitflag = ',num2str(exitflag)]);
end

%% show results
param

tau_est = -cost_single_f14(param,sdata{meas}.q0_s{set},zeros(size(sdata{meas}.q0_s{set})),sdata{meas}.q2);
figure(fig1);
plot(sdata{meas}.q0_s{set},tau_est);