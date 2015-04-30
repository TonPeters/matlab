clear all; 
close all; 
clc;
%-------------------------------------------------------------------------%
% Model: Trunk
%-------------------------------------------------------------------------%
set(0,'defaulttextinterpreter','latex');
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
    for sset = sets
        figure(fig1);
        if sset<3
            subplot(1,2,1); 
        else
            subplot(1,2,2);
        end
        plot(sdata{meas}.q0_m{sset},sdata{meas}.tau_m{sset},'color',ps.list_div{meas,1},'linewidth',0.5); hold all;
%         plot(sdata{meas}.q0_s{set},sdata{meas}.tau_f{set},'--','color',ps.list_div{meas,2});
        
        % combine data
        dat_tau = [dat_tau;sdata{meas}.tau_f{sset}];
        dat_q0 = [dat_q0;sdata{meas}.q0_s{sset}];
        dat_qd0 = [dat_qd0;ones(size(sdata{meas}.q0_s{sset})).*sdata{meas}.qd0{sset}];
        dat_q2 = [dat_q2;ones(size(sdata{meas}.q0_s{sset})).*sdata{meas}.q2];
    end
    sset = 1;

    %% optimization
%     m_1= param(1);
%     m_2= param(2);
    param0 =    [3.5,   5];
    lb =        [3,   1.5];
    ub =        [9,     9];

    % lsqnonlin
    opts = optimset('lsqnonlin');
    opts = optimset(opts,'tolfun',1E-16,'display','off','tolx',1E-16);
    problem = createOptimProblem('lsqnonlin','objective',...
        @(param)cost_legs_fricLS_mass(param,dat_q0,dat_tau,dat_q2,dat_qd0),...
        'x0',param0,'lb',lb,'ub',ub,'options',opts);
    ms = MultiStart('Display','off');
    n_startpoints = 2;
    [param,fval,exitflag] = run(ms,problem,n_startpoints);

    if (exitflag ~= 1)
        display(['No solution, exitflag = ',num2str(exitflag)]);
    end

    params{meas,sset} = param;
    %% show results

end

%% check params
par = zeros(length(measurements),length(param));
for meas = measurements
    par(meas,:) = params{meas,sset};
end
par
avg_par = mean(par)


for meas = measurements
    % model calculation
    tau_est_p{meas,sset} = -cost_legs_fricLS_mass(avg_par,q0_est,zeros(size(q0_est)),sdata{meas}.q2,0.004);
    tau_est_n{meas,sset} = -cost_legs_fricLS_mass(avg_par,q0_est,zeros(size(q0_est)),sdata{meas}.q2,-0.004);
    
    % plot fit
    figure(fig1);
    ax1 = subplot(1,2,1);
    pl{meas} = plot(q0_est,tau_est_p{meas,sset},'color',ps.list_div{meas,2});
    ax2 = subplot(1,2,2);
    plot(q0_est,tau_est_n{meas,sset},'color',ps.list_div{meas,2});
end

%% report
figure(fig1);
axes(ax2);
linkaxes([ax1 ax2],'xy')
ylim([0.01 0.1]); 
xlim([th_0_min th_0_max]);
% xlim([0.25 1]);
title('Negative velocity');
set(ax2,'yticklabel',[]);
xlabel('joint angle $q_0$ [rad]');
axes(ax1)
title('Positive velocity');
ylabel('$\tau$ [Nm]');
xlabel('joint angle $q_0$ [rad]');
leg = legend([pl{1},pl{2},pl{3}],{'$q_2=1.01$ rad','$q_2=1.4$ rad','$q_2=1.6$ rad'},'location','southeast');
set(leg,'interpreter','latex');
setplot


all_grids_on();


%% save report
dir = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/';

save_report(fig1,dir,'fit_legs2','paperwidth');









all_grids_on();