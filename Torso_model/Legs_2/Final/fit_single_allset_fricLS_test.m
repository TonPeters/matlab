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
load('data_legs.mat');
measurements = [1,2,3];
sets = [1,2,3,4];
i=1;
q0_est = linspace(th_0_min,th_0_max,40).';


fig1 = figure; scr rt;
fig2 = figure; scr lt;
for meas = measurements
    
    figure(fig2); 
    plot(sdata{meas}.q0_s{1},((sdata{meas}.tau_f{1}+sdata{meas}.tau_f{2})./2-(sdata{meas}.tau_f{3}+sdata{meas}.tau_f{3})./2),'color',ps.list_div{meas,2}); hold all;
    
    dat_tau = [];
    dat_qd0 = [];
    dat_q2 = [];
    dat_q0 = [];
    for set = sets
        figure(fig1);
        plot(sdata{meas}.q0_m{set},sdata{meas}.tau_m{set},'color',ps.list_div{meas,1}); hold all;
        plot(sdata{meas}.q0_s{set},sdata{meas}.tau_f{set},'--','color',ps.list_div{meas,2});
        
        % combine data
        dat_tau = [dat_tau;sdata{meas}.tau_f{set}];
        dat_q0 = [dat_q0;sdata{meas}.q0_s{set}];
        dat_qd0 = [dat_qd0;ones(size(sdata{meas}.q0_s{set})).*sdata{meas}.qd0{set}];
        dat_q2 = [dat_q2;ones(size(sdata{meas}.q0_s{set})).*sdata{meas}.q2];
    end
    set = 1;

    %% optimization
    
    m_1mm = [2; 9]; 
    m_2_mm = [2; 9];
    
    lcm3_m3 = 2.4724;
    m_3 = lcm3_m3/l_cm3;
    P1_est1 = l_cm1*m_1 + l_1*m_2 + l_1*m_3 + l_1*m_4;
    P2_est1 = l_cm2*m_2 + l_2*m_3 + l_2*m_4;
    P1_bound = l_cm1*m_1mm + l_1*m_2_mm + l_1*m_3 + l_1*m_4;
    P2_bound = l_cm2*m_2_mm + l_2*m_3 + l_2*m_4;    

    P1_est = [P1_est1;  P1_bound];    % P1 = l_cm1*m_1 + l_1*m_2 + l_1*m_3 + l_1*m_4;
    P2_est = [P2_est1;  P2_bound];    % P2 = l_cm2*m_2 + l_2*m_3 + l_2*m_4;
    P3_est = [0.1;  0;  10];    % Kls
    P4_est = [0.1;  -1;  1];    % Kdir
    P5_est = [0.1;  0;  10];    % Kcoul

    P_est = [P3_est,P4_est,P5_est];

    param0 =    P_est(1,:);
    lb =        P_est(2,:);
    ub =        P_est(3,:);

    % lsqnonlin
    opts = optimset('lsqnonlin');
    opts = optimset(opts,'tolfun',1E-16,'display','off','tolx',1E-16);
    problem = createOptimProblem('lsqnonlin','objective',...
        @(param)cost_legs_fricLS_test(param,dat_q0,dat_tau,dat_q2,dat_qd0),...
        'x0',param0,'lb',lb,'ub',ub,'options',opts);
    ms = MultiStart('Display','off');
    n_startpoints = 2;
    [param,fval,exitflag] = run(ms,problem,n_startpoints);

    if (exitflag ~= 1)
        display(['No solution, exitflag = ',num2str(exitflag)]);
    end

    params{meas,set} = param;
    %% show results

    % model calculation
    tau_est_p{meas,set} = -cost_legs_fricLS_test(param,q0_est,zeros(size(q0_est)),sdata{meas}.q2,0.004);
    tau_est_n{meas,set} = -cost_legs_fricLS_test(param,q0_est,zeros(size(q0_est)),sdata{meas}.q2,-0.004);

    figure(fig1);
    plot(q0_est,tau_est_p{meas,set},'color',ps.list_div{meas,2});
    plot(q0_est,tau_est_n{meas,set},'color',ps.list_div{meas,2});
end

% tau_mass = -cost_legs_fricLS([0,0,0],q0_est,zeros(size(q0_est)),sdata{meas}.q2,0.004);
% plot(q0_est,tau_mass,'color',ps.tuedarkblue);

%% check params
par = zeros(length(measurements),length(param));
for meas = measurements
    par(meas,:) = params{meas,set};
end
par
avg_par = mean(par)

figure; 
subplot(3,1,1);
plot(par(:,1));
subplot(3,1,2);
plot(par(:,2));
subplot(3,1,3);
plot(par(:,3));

all_grids_on();