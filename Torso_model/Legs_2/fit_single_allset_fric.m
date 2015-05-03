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
fig2 = figure; scr lt;
for meas = measurements
    
    figure(fig2); 
    plot(sdata{meas}.q0_s{1},((sdata{meas}.tau_f{1}+sdata{meas}.tau_f{2})./2-(sdata{meas}.tau_f{3}+sdata{meas}.tau_f{3})./2),'color',ps.list_div{meas,2}); hold all;
    for set = sets
        figure(fig1);
        plot(sdata{meas}.q0_m{set},sdata{meas}.tau_m{set},'color',ps.list_div{meas,1}); hold all;
        plot(sdata{meas}.q0_s{set},sdata{meas}.tau_f{set},'--','color',ps.list_div{meas,2});

        %% optimization
        param0 =    [1,1];
        lb =        [0,-1000];
        ub =        [1000,1000];

        % lsqnonlin
        opts = optimset('lsqnonlin');
        opts = optimset(opts,'tolfun',1E-16,'display','off','tolx',1E-16);
        problem = createOptimProblem('lsqnonlin','objective',...
            @(param)cost_single_fric(param,sdata{meas}.q0_s{set},sdata{meas}.tau_f{set},sdata{meas}.q2,sdata{meas}.qd0{set}),...
            'x0',param0,'lb',lb,'ub',ub,'options',opts);
        ms = MultiStart('Display','off');
        n_startpoints = 2;
        [param,fval,exitflag] = run(ms,problem,n_startpoints);

        if (exitflag ~= 1)
            display(['No solution, exitflag = ',num2str(exitflag)]);
        end

        params{meas,set} = param;
        %% show results
        param;
        P1= param(1);   % fric proportional 
%         P2 = param(1);                    % P1 = lcm1*m1 + l1*m2 + l1*m3 + l1*m4
%         P3 = param(2);                    % P2 = lcm2*m2 + l2*m3 + l2*m4
%         P4 = 2.4724;                      % P4 = lcm3*m3 +l3*m4
        P5 = param(2);  % fric const


        % model calculation
        tau_est{meas,set} = -cost_single_fric(param,q0_est,zeros(size(q0_est)),sdata{meas}.q2,sdata{meas}.qd0{set});

        figure(fig1);
        plot(q0_est,tau_est{meas,set},'color',ps.list_div{meas,2});

    end
    title(['q2 ',num2str(sdata{meas}.q2)])
end
figure(fig2);
thF0 = spindle1_to_Fangle1(angle0_to_spindle1(q0_est));
scale = 0.1;
plot(q0_est,sin(thF0).*scale,'color',ps.tuedarkblue);


%% check params
par = zeros(length(measurements),length(sets),length(param));
par_p = zeros(length(measurements),length(sets)./2,length(param));
par_n = zeros(length(measurements),length(sets)./2,length(param));
for meas = measurements
    for set = sets
        par(meas,set,:) = params{meas,set};
        if set<3
            par_p(meas,set,:) = params{meas,set};
        else
            par_n(meas,set-2,:) = params{meas,set};
        end
    end
end

% average positive
avg_p = mean(mean(par_p))
avg_n = mean(mean(par_n))




all_grids_on();