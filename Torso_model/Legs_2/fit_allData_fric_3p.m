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
clearvars -except g l_1 l_2 l_3 l_F1 l_F2 th_2_min th_2_max th_0_min th_0_max g m_1 m_2 m_3 m_4 l_cm1 l_cm2
plotsettings

%% Load data
load('data_legs.mat');
measurements = [1,2,3];
sets = [1,2,3,4];
i=1;
q0_est = linspace(th_0_min,th_0_max,40).';


fig1 = figure; scr rt;
for meas = measurements
    dat_q0 = [];
    dat_qd0 = [];
    dat_tau = [];
    
    for set = sets
        plot(sdata{meas}.q0_m{set},sdata{meas}.tau_m{set},'color',ps.list_div{meas,1}); hold all;
        plot(sdata{meas}.q0_s{set},sdata{meas}.tau_f{set},'--','color',ps.list_div{meas,2});
        
        dat_q0 = [dat_q0;  sdata{meas}.q0_s{set}];  
        dat_tau = [dat_tau; sdata{meas}.tau_f{set}];
        dat_qd0 = [dat_qd0; ones(size(sdata{meas}.q0_s{set})).*sdata{meas}.qd0{set}];

    end
    set = 1;
    %% optimization
    param0 =    [1,1,1];
    lb =        [0,0,-1000];
    ub =        [1000,1000,1000];

    % lsqnonlin
    opts = optimset('lsqnonlin');
    opts = optimset(opts,'tolfun',1E-16,'display','off','tolx',1E-16);
    problem = createOptimProblem('lsqnonlin','objective',...
        @(param)cost_single_fric_p3(param,dat_q0,dat_tau,sdata{meas}.q2,dat_qd0),...
        'x0',param0,'lb',lb,'ub',ub,'options',opts);
    ms = MultiStart('Display','off');
    n_startpoints = 4;
    [param,fval,exitflag] = run(ms,problem,n_startpoints);

    if (exitflag ~= 1)
        display(['No solution, exitflag = ',num2str(exitflag)]);
    end

    params{meas,set} = param;
    %% show results
    param;
    P1 = param(1);
%     P2 = param(1);                    % P1 = lcm1*m1 + l1*m2 + l1*m3 + l1*m4
%     P3 = param(2);                    % P2 = lcm2*m2 + l2*m3 + l2*m4
%     P4 = 2.4724;                      % P4 = lcm3*m3 +l3*m4
    P5 = param(2);                    % coul_fric;
    P6 = param(3);                    % coul_fric;


    % model calculation
    tau_est{meas,1} = -cost_single_fric_p3(param,q0_est,zeros(size(q0_est)),sdata{meas}.q2,0.004);
    tau_est{meas,2} = -cost_single_fric_p3(param,q0_est,zeros(size(q0_est)),sdata{meas}.q2,-0.004);

    figure(fig1);
    plot(q0_est,tau_est{meas,1},'color',ps.list_div{meas,2});
    plot(q0_est,tau_est{meas,2},'color',ps.list_div{meas,2});
%     title(['q2 ',num2str(sdata{meas}.q2)])
end

%% check params
par = zeros(length(measurements),length(param));
% par_p = zeros(length(measurements),length(param));
% par_n = zeros(length(measurements),length(sets)./2,length(param));
for meas = measurements
%     for set = sets
        par(meas,:) = params{meas,1};
%         if set<3
%             par_p(meas,set,:) = params{meas,set};
%         else
%             par_n(meas,set-2,:) = params{meas,set};
%         end
%     end
end
% 
% % positive ratio
% ratio_p = par_p(:,:,1)./par_p(:,:,2);
% ratio_p_mean = mean(mean(ratio_p))
% 
% % negative ratio
% ratio_n = par_n(:,:,1)./par_n(:,:,2);
% ratio_n_mean = mean(mean(ratio_n))
% 
% % average positive
% avg_p = mean(mean(par_p))
% avg_n = mean(mean(par_n))




all_grids_on();