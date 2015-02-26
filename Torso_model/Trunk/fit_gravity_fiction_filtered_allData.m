clear all; 
close all; 
clc;
%-------------------------------------------------------------------------%
% Model: Trunk
%-------------------------------------------------------------------------%

%% get measures from model
run torso_measures_NX

% m1 = M_LRL+M_LFL;
% m2 = M_UL;
% m3 = 20;        % mass of the trunk
lAG = di(A,G);
lAE = di(A,E);
lAC = di(A,C);
lGJ = di(G,J);
lJK = di(J,K);
lJH = di(J,H);
lDJ = di(D,J);
lJL = di(J,L);
aBAZ = an(B,A,A+[0;-1;0]);
th_leg = 0.56;
th_leg2 = 0.2;
g = 9.81;

th_2_min = spindle2_to_angle2(min_spindle2);
th_2_max = spindle2_to_angle2(max_spindle2);
l1 = lJL;
lcm1 = l1*0.7;
lF = lJK;
th_r = -th_leg+angle0_to_angle1(th_leg);
th_r2 = -th_leg2+angle0_to_angle1(th_leg2);
m1 = 8;
% m2 = 0;
m1_lcm1 = 2.63;

% clear parameters
clearvars -except l1 lcm1 lF th_r m1 th_2_min th_2_max g m1_lcm1 th_r2
plotsettings
%% load data to fit
PC = 'ton/git_ton';
tau_grav_shift = -0.033;
% tau_grav_shift = 0.0;
leg_list = {'m0 c','m10 c','m20 c','m0 d','m10 d'};
leg_list2 = {}; leg_count = 1;
q2 = linspace(th_2_min,th_2_max,20).';
n_tot = 0;

    
fig1 = figure;scr lt; 
% fig2 = figure; scr rt;
% fig3 = figure; scr rb;
fig11 = figure; scr rt;




params = [];
for i=1:1:5
%     if i==3
%         continue;
%     end
    leg_list2{leg_count} = leg_list{i}; leg_count= leg_count+1;
    
    % 0kg 0.56rad
    if i==1
        load(['/home/',PC,'/matlab/Torso_Identification/add_mass/upper_3mass.mat']);
        data_angle  = th_0k; % measured angle
        data_offset{i} = th_r; % angle offset
        data_input  = u_0k; % control input
        m2{i}          = 0;
        q_min       = 1.1;  % minimum angle
        q_max       = 2.2;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 32*1000+1;   % center switching velocity
        vel_stop    = 64*1000+1;   % end negative velocity
    elseif i==2
    % 10kg 0.56rad
        load(['/home/',PC,'/matlab/Torso_Identification/add_mass/upper_3mass.mat']);
        data_angle  = th_10k; % measured angle
        data_offset{i} = th_r; % angle offset
        data_input  = u_10k; % control input
        m2{i}          = 10;
        q_min       = 1.16;  % minimum angle
        q_max       = 2.31;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 28*1000+1;   % center switching velocity
        vel_stop    = 58*1000+1;   % end negative velocity
    elseif i==3
    % 20kg 0.56rad
        load(['/home/',PC,'/matlab/Torso_Identification/add_mass/upper_3mass.mat']);
        data_angle  = th_20k; % measured angle
        data_offset{i} = th_r; % angle offset
        data_input  = u_20k; % control input
        m2{i}          = 20;
        q_min       = 1.6;  % minimum angle
        q_max       = 2.3;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 25*1000+1;   % center switching velocity
        vel_stop    = 60*1000+1;   % end negative velocity
    elseif i==4
    % 0kg 0.2rad
        load(['/home/',PC,'/matlab/Torso_Identification/add_mass/upper_2mass_leg02.mat']);
        data_angle  = th_0k2; % measured angle
        data_offset{i} = th_r2; % angle offset
        data_input  = u_0k2; % control input
        m2{i}          = 0;
        q_min       = 1.1;  % minimum angle
        q_max       = 2.3;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 28*1000+1;   % center switching velocity
        vel_stop    = 53*1000+1;   % end negative velocity
    elseif i==5
    % 10kg 0.2rad
        load(['/home/',PC,'/matlab/Torso_Identification/add_mass/upper_2mass_leg02.mat']);
        data_angle  = th_10k2; % measured angle
        data_offset{i} = th_r2; % angle offset
        data_input  = u_10k2; % control input
        m2{i}          = 10;
        q_min       = 1.1;  % minimum angle
        q_max       = 2.28;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 27*1000+1;   % center switching velocity
        vel_stop    = 53*1000+1;   % end negative velocity
    end
    
    % set data sample points
    n_s = 1000; % number of samples used for fitting
    q_m_s{i} = linspace(q_min+0.01,q_max-0.01,n_s).';


    % figure
    % subplot(2,1,1); plot(data_angle); grid on;
    % subplot(2,1,2); plot(data_angle./180*pi+data_offset,data_input); grid on;
    % pause


    %% use data
    q_meas{i} = data_angle./180*pi+data_offset{i}; 
%     qF0 = spindle2_to_Fangle2(angle2_to_spindle2(q_meas{i}));
    tau_meas{i} = data_input;

    % compute velocity
    q_m = q_meas{i}(vel_start:vel_stop);
    tau_m = tau_meas{i}(vel_start:vel_stop);
    qd_m = ones(size(q_m));
    qd_m(vel_switch:end) = -1;

    % cut data
    indi_p = find(q_m>q_min & q_m<q_max);
    q_m = q_m(indi_p);
    tau_m = tau_m(indi_p);
    qd_m = qd_m(indi_p);
    % figure; plot(q_m,tau_m,'.');
    % figure; plot(qd_m,tau_m,'.');

    %% filter data

    % lowpass filter using butterworth
    indi_p = find(qd_m>0);
    tau_m_filt_p = filter_data(tau_m(indi_p),4,0.1);
    indi_n = find(qd_m<0);
    tau_m_filt_n = filter_data(tau_m(indi_n),4,0.1);

    % remove datapoints that have the same encoder position and sample data
    [q_m_f,IA,IC] = unique(q_m(indi_p));
    tau_p = interp1(q_m_f,tau_m_filt_p(IA),q_m_s{i});
    [q_m_f,IA,IC] = unique(q_m(indi_n));
    tau_n = interp1(q_m_f,tau_m_filt_n(IA),q_m_s{i});

    % compute gravity term
    tau_grav{i} = (tau_p+tau_n)./2+tau_grav_shift;

    % compute friction term
    tau_fric_p{i} = tau_p-(tau_grav{i}-tau_grav_shift);
    tau_fric_n{i} = tau_n-(tau_grav{i}-tau_grav_shift);    


    figure(fig11);
    % plot((q_m-data_offset)./pi*180,tau_m,':','color',ps.list{i}); hold all;
    plot((q_m_s{i}-data_offset{i})./pi*180,tau_grav{i},'color',ps.list{i}); hold all;
    plot((q_m_s{i}-data_offset{i})./pi*180,tau_fric_p{i},'--','color',ps.list{i}); hold all;
    plot((q_m_s{i}-data_offset{i})./pi*180,tau_fric_n{i},'-.','color',ps.list{i}); hold all;

    n_tot = n_tot+length(q_m_s{i});
end

tau_fric_p_mean =  mean([tau_fric_p{1};tau_fric_p{2};tau_fric_p{3};tau_fric_p{4};tau_fric_p{5}])
tau_fric_n_mean =  mean([tau_fric_n{1};tau_fric_n{2};tau_fric_n{3};tau_fric_n{4};tau_fric_n{5}])
figure(fig11);
plot((q2-data_offset{i})./pi*180,ones(size(q2)).*tau_fric_p_mean,'color',ps.list{1},'linewidth',ps.linewidth); hold all;
plot((q2-data_offset{i})./pi*180,ones(size(q2)).*tau_fric_n_mean,'color',ps.list{1},'linewidth',ps.linewidth); hold all;
    


q_tot = zeros(n_tot,1);
qd_tot = q_tot;
tau_tot = q_tot;
m2_tot = q_tot;
data_offset_tot = q_tot;
i_start = 1;
for i=1:1:5
    ni = length(q_m_s{i});
    i_end = i_start+ni-1;
    q_tot(i_start:i_end) = q_m_s{i};
    tau_tot(i_start:i_end) = tau_grav{i};
    m2_tot(i_start:i_end) = m2{i}.*ones(ni,1);
    data_offset_tot(i_start:i_end) = data_offset{i}.*ones(ni,1);
    
    i_start = i_end+1;
end



%% optimization fit
% initial gues
param0 = [2.*l1/4];

lb = [2.*l1/4];
ub = [10*l1];
opts = optimset('lsqnonlin');
opts = optimset(opts,'tolfun',1E-16,'display','off','tolx',1E-16);
% [param,resnorm,residual,exitflag] = lsqnonlin(@(param)costfunc_friction_gravity_nonlin(param,q_m,qd_m,tau_m,m2,data_offset),param0,lb,ub,opts);
% exitflag    
% param

problem = createOptimProblem('lsqnonlin','objective',...
    @(param)costfunc_gravity_nonlin_V1(param,q_tot,tau_tot,m2_tot,data_offset_tot),...
    'x0',param0,'lb',lb,'ub',ub,'options',opts);
ms = MultiStart('Display','off');
n_startpoints = 2;
[param,fval,exitflag] = run(ms,problem,n_startpoints);
if (exitflag ~= 1)
    display(['No solution, exitflag = ',num2str(exitflag)]);
end

% params = [2.3492    2.5668    2.3828    2.5906];
% param_mean = 2.4724
m1_lcm1 = param(1);

for i=1:1:5
    params = [params,param];
    %% compare fit to measurement
    % Parameters
    l_ls = 0.002;
    rsp = 2*pi/l_ls;
    rgear = 13/3;
    Kmm = 29.2e-3;             % Nm/A,     torque constant
    Kelm = 10;

    % V_mod = ((g*(m1_lcm1+m2*l1)*cos(q_m-th_r))./lF./sin(thF) + Kc*sign(qd_m)+Kd)./rsp/rgear/Kmm/Kelm;

    % Moment needed
    M_joint = (g.*(m1_lcm1+m2{i}*l1).*cos(q2-data_offset{i}+2/180*pi));

    % Effective force needed
    F_eff = M_joint./lF;

    % applied force needed
    % thF = -0.9902.*q_m+3.152;     % Angle force applied
    thF = spindle2_to_Fangle2(angle2_to_spindle2(q2));
    F_appl = F_eff./sin(thF);

    % Spindle torque needed
    T_sp = F_appl/rsp;

    % Motor torque needed
    T_m = T_sp/rgear;

    % Motor current needed
    I_m = T_m/Kmm;

    % Input Voltage needed
    V_mod = I_m/Kelm;

    % plot results
    figure(fig1);
%     figure; scr rt;
    % subplot(2,1,1)
    plot((q_meas{i}-data_offset{i})./pi*180,tau_meas{i},':','color',ps.list{i}); hold all;
    plot((q_m_s{i}-data_offset{i})./pi*180,tau_grav{i},'--','color',ps.list{i}); hold all;
    plot((q2-data_offset{i})./pi*180,V_mod,'color',ps.list{i});
    ylabel('model error +vel');
    % subplot(2,1,2)
    % plot((q_m(indi_n)-data_offset)./pi*180,tau_m(indi_n),'color',ps.list{i}); hold all;
    % plot((q_m(indi_p)-data_offset)./pi*180,tau_test(indi_p),':','color',ps.list{i}); hold all;
    % plot((q_m(indi_n)-data_offset)./pi*180,V_mod(indi_n),'color',ps.list{i});
    % ylabel('model error -vel');

    % figure(fig2);
    % % F1= fit(q_m(indices)-data_offset,tau_m(indices)-V_mod(indices),'poly3');
    % % F2= fit(q_m(indices2)-data_offset,tau_m(indices2)-V_mod(indices2),'poly3');
    % subplot(2,1,1)
    % plot((q_m(indi_p)-data_offset)./pi*180,tau_m(indi_p)-V_mod(indi_p),'color',ps.list{i}); hold all;
    % ylabel('model error +vel');
    % legend(leg_list2)
    % subplot(2,1,2)
    % plot((q_m(indi_n)-data_offset)./pi*180,tau_m(indi_n)-V_mod(indi_n),'color',ps.list{i}); hold all;
    % ylabel('model error -vel');

    % Invest gravity torque
    % M_joint_meas = tau_m.*(rsp*rgear*Kmm*Kelm).*sin(thF).*lF;

    % figure(fig3);
    % subplot(2,1,1)
    % plot((q_m(indices)-data_offset)./pi*180,M_joint_meas(indices),'color',ps.list{i}); hold all;
    % ylabel('Moment [Nm] +vel');
    % legend(leg_list2)
    % subplot(2,1,2)
    % plot((q_m(indices2)-data_offset)./pi*180,M_joint_meas(indices2),'color',ps.list{i}); hold all;
    % ylabel('Moment [Nm] -vel');

end
% linkaxes(get(fig2,'children'),'x');
params


% end
all_grids_on();
