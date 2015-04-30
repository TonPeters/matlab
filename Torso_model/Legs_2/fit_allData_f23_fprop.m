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
fig2 = figure; scr lt;
dat_tau2 = [];
dat_q0 = [];
dat_q2 = [];
for meas = measurements
%     for set = sets
        set = 1;
        figure(fig1);
%         plot(sdata{meas}.q0_m{set},sdata{meas}.tau_m{set},'color',ps.list_div{meas,1}); hold all;
%         plot(sdata{meas}.q0_s{set},sdata{meas}.tau_f{set},'--','color',ps.list_div{meas,2});

        dat_tau = zeros(size(sdata{meas}.q0_s{set}));
        dat_tau_diff = dat_tau;
%         dat_qd0 = dat_q0;
        for set = sets
            figure(fig1);
            plot(sdata{meas}.q0_m{set},sdata{meas}.tau_m{set},'color',ps.list_div{meas,1}); hold all;
            plot(sdata{meas}.q0_s{set},sdata{meas}.tau_f{set},'--','color',ps.list_div{meas,2});
            
%             dat_q0 = dat_q0+  sdata{meas}.q0_s{set}];  
            dat_tau = dat_tau+ sdata{meas}.tau_f{set};
%             dat_qd0 = dat_qd0+ ones(size(sdata{meas}.q0_s{set})).*sdata{meas}.qd0{set};
            if set<3
%                 disp('p')
                dat_tau_diff = dat_tau_diff+sdata{meas}.tau_f{set};
            else
%                 disp('n')
                dat_tau_diff = dat_tau_diff-sdata{meas}.tau_f{set};
            end
        end
        % plot avg
        dat_tau = dat_tau./4;
        plot(sdata{meas}.q0_s{set},dat_tau,'color',ps.list_div{meas,1});
        
        % plot diff
        dat_tau_diff = dat_tau_diff./2;
        figure(fig2);
        plot(sdata{meas}.q0_s{set},dat_tau_diff,'color',ps.list_div{meas,2}); hold all;
        
        %% friction
        figure(fig2)
        Ffric = fit(sdata{meas}.q0_s{set},dat_tau_diff,'poly1');
        plot(sdata{meas}.q0_s{set},feval(Ffric,sdata{meas}.q0_s{set}))
        tau_coul = mean(dat_tau_diff)./2;
        tau_coul2 = Ffric.p2/2;
        tau_pos = Ffric.p1/2;
        
        
        %% data to fit
        dat_tau2 = [dat_tau2;dat_tau];
        dat_q0 = [dat_q0;sdata{meas}.q0_s{set}];
        dat_q2 = [dat_q2;ones(size(dat_tau)).*sdata{meas}.q2];
end
set = 1;


%% optimization
param0 =    [l_cm1*m_1+l_1*(m_2+m_3+m_4) ,l_cm2*m_2+l_2*(m_3+m_4), 1.1 ];
lb =        [0,0,0];
ub =        [10,10,10];

% lsqnonlin
opts = optimset('lsqnonlin');
opts = optimset(opts,'tolfun',1E-16,'display','off','tolx',1E-16);
problem = createOptimProblem('lsqnonlin','objective',...
    @(param)cost_single_f23_fprop(param,dat_q0,dat_tau2,dat_q2),...
    'x0',param0,'lb',lb,'ub',ub,'options',opts);
ms = MultiStart('Display','off');
n_startpoints = 2;
[param,fval,exitflag] = run(ms,problem,n_startpoints);

if (exitflag ~= 1)
    display(['No solution, exitflag = ',num2str(exitflag)]);
end

params{meas,set} = param;
disp('params estimated');
%% show results

for meas = measurements
        % model calculation
        tau_est{meas,set} = -cost_single_f23_fprop(param,q0_est,zeros(size(q0_est)),sdata{meas}.q2);

        figure(fig1);
        plot(q0_est,tau_est{meas,set},'color',ps.list_div{meas,2});
        plot(q0_est,tau_est{meas,set}+tau_coul,'color',ps.list_div{meas,2});
        plot(q0_est,tau_est{meas,set}-tau_coul,'color',ps.list_div{meas,2});

%     end
    title(['q2 ',num2str(sdata{meas}.q2)])
end

%% check params
par = zeros(length(measurements),length(param));
% par_p = zeros(length(measurements),length(sets)./2,length(param));
% par_n = zeros(length(measurements),length(sets)./2,length(param));
for meas = measurements
%     for set = sets
        par(meas,:) = params{meas,set};
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
% 
par


all_grids_on();