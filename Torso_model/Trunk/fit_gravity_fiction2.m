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

th_0_min = spindle2_to_angle2(min_spindle2);
th_0_max = spindle2_to_angle2(max_spindle2);
l1 = lJL;
lcm1 = l1*0.7;
lF = lJK;
th_r = -th_leg+angle0_to_angle1(th_leg);
th_r2 = -th_leg2+angle0_to_angle1(th_leg2);
m1 = 8;
m2 = 0;
m1_lcm1 = 2.63;

% clear parameters
clearvars -except l1 lcm1 lF th_r m1 m2 th_0_min th_0_max g m1_lcm1 th_r2
plotsettings
%% load data to fit
% load '/home/ton/git_ton/matlab/Torso_Identification/add_mass/upper_3mass.mat'
fig1 = figure;scr lt; fig2 = figure; scr rt;
fig3 = figure; scr rb;

leg_list = {'m0 c','m10 c','m20 c','m0 d','m10 d'};
leg_list2 = {}; leg_count = 1;
params = zeros(3,5);
for i=1:5

    leg_list2{leg_count} = leg_list{i}; leg_count= leg_count+1;
    
    % 0kg 0.56rad
    if i==1
        load '/home/amigo/matlab/Torso_Identification/add_mass/upper_3mass.mat'
        data_angle  = th_0k; % measured angle
        data_offset = th_r; % angle offset
        data_input  = u_0k; % control input
        m2          = 0;
        q_min       = 1.1;  % minimum angle
        q_max       = 2.2;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 32*1000+1;   % center switching velocity
        vel_stop    = 64*1000+1;   % end negative velocity
    elseif i==2
    % 10kg 0.56rad
        load '/home/amigo/matlab/Torso_Identification/add_mass/upper_3mass.mat'
        data_angle  = th_10k; % measured angle
        data_offset = th_r; % angle offset
        data_input  = u_10k; % control input
        m2          = 10;
        q_min       = 1.16;  % minimum angle
        q_max       = 2.31;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 28*1000+1;   % center switching velocity
        vel_stop    = 58*1000+1;   % end negative velocity
    elseif i==3
    % 20kg 0.56rad
        load '/home/amigo/matlab/Torso_Identification/add_mass/upper_3mass.mat'
        data_angle  = th_20k; % measured angle
        data_offset = th_r; % angle offset
        data_input  = u_20k; % control input
        m2          = 20;
        q_min       = 1.6;  % minimum angle
        q_max       = 2.3;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 25*1000+1;   % center switching velocity
        vel_stop    = 60*1000+1;   % end negative velocity
    elseif i==4
    % 0kg 0.2rad
        load '/home/amigo/matlab/Torso_Identification/add_mass/upper_2mass_leg02.mat'
        data_angle  = th_0k2; % measured angle
        data_offset = th_r2; % angle offset
        data_input  = u_0k2; % control input
        m2          = 0;
        q_min       = 1.1;  % minimum angle
        q_max       = 2.3;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 28*1000+1;   % center switching velocity
        vel_stop    = 53*1000+1;   % end negative velocity
    elseif i==5
    % 10kg 0.2rad
        load '/home/amigo/matlab/Torso_Identification/add_mass/upper_2mass_leg02.mat'
        data_angle  = th_10k2; % measured angle
        data_offset = th_r2; % angle offset
        data_input  = u_10k2; % control input
        m2          = 10;
        q_min       = 1.1;  % minimum angle
        q_max       = 2.28;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 27*1000+1;   % center switching velocity
        vel_stop    = 53*1000+1;   % end negative velocity
    end
    




% figure
% subplot(2,1,1); plot(data_angle); grid on;
% subplot(2,1,2); plot(data_angle./180*pi+data_offset,data_input); grid on;
% pause


%% use data
q_meas = data_angle./180*pi+data_offset; qF0 = spindle2_to_Fangle2(angle2_to_spindle2(q_meas));
tau_meas = data_input;

% compute velocity
q_m = q_meas(vel_start:vel_stop);
tau_m = tau_meas(vel_start:vel_stop);
qd_m = ones(size(q_m));
qd_m(vel_switch:end) = -1;

% cut data
indices = find(q_m>q_min & q_m<q_max);
q_m = q_m(indices);
tau_m = tau_m(indices);
qd_m = qd_m(indices);
% figure; plot(q_m,tau_m,'.');
% figure; plot(qd_m,tau_m,'.');




%% optimization fit
% initial gues
param0 = [2.*l1/4;  400;   100];

% % [param,resnorm,exitflag] = fminunc(@(param)costfunc_friction_gravity2(param,q_m,qd_m,tau_m,m2,data_offset),param0)
% 
% lb = [2.*l1/4; 10; 10];
% ub = [8*l1; 10000; 10000];
% opts = optimset('lsqnonlin');
% opts = optimset(opts,'tolfun',1E-16,'display','off','tolx',1E-16);
% % [param,resnorm,residual,exitflag] = lsqnonlin(@(param)costfunc_friction_gravity_nonlin(param,q_m,qd_m,tau_m,m2,data_offset),param0,lb,ub,opts);
% % exitflag    
% % param
% 
% problem = createOptimProblem('lsqnonlin','objective',...
%     @(param)costfunc_friction_gravity_nonlin(param,q_m,qd_m,tau_m,m2,data_offset),...
%     'x0',param0,'lb',lb,'ub',ub,'options',opts);
% ms = MultiStart('Display','off');
% n_startpoints = 2;
% [param,fval,exitflag] = run(ms,problem,n_startpoints)
    
param = [2.7955  394.8329   91.9194];

% m1_lcm1 = 2.63;
% Kc = 0;
% Kd = 0;

% params total
%     2.2914    2.7229    3.7236    2.4522    2.7874
%   394.1316  391.5851  408.6118  387.9323  391.9037
%   128.1708   94.0193   39.4573  114.0061   83.9436
% param = [2.7955  394.8329   91.9194];
% params positive velocity
%     2.2404    2.3965    2.8949    2.0238    1.7924
%   414.2289  403.0824  388.4929  407.4627  401.6814
%   111.3154  100.1689   85.5794  104.5492   98.7679
% param = [2.2696  402.9896  100.0761];
% params negative velocity
%     2.3423    3.0491    4.5766    2.8807    3.7821
%    16.8553   -6.1422  -46.2874    9.4562  -14.8066
%  -286.0577 -309.0561 -349.2010 -293.4580 -317.7203
% param = [3.3262   -8.1849 -311.0986];


m1_lcm1 = param(1);
Kc = param(2);
Kd = param(3);

params(:,i) = param;
%% compare fit to measurement
% Parameters
l_ls = 0.002;
rsp = 2*pi/l_ls;
rgear = 13/3;
Kmm = 29.2e-3;             % Nm/A,     torque constant
Kelm = 10;

% V_mod = ((g*(m1_lcm1+m2*l1)*cos(q_m-th_r))./lF./sin(thF) + Kc*sign(qd_m)+Kd)./rsp/rgear/Kmm/Kelm;

% Moment needed
M_joint = (g*(m1_lcm1+m2*l1)*cos(q_m-data_offset));

% Effective force needed
F_eff = M_joint./lF;

% applied force needed
% thF = -0.9902.*q_m+3.152;     % Angle force applied
thF = spindle2_to_Fangle2(angle2_to_spindle2(q_m));
F_appl = F_eff./sin(thF);
    
% add Friction force
Ffr = Kc*sign(qd_m)+Kd;
F_appl_fric = F_appl+Ffr;

% Spindle torque needed
T_sp = F_appl_fric/rsp;

% Motor torque needed
T_m = T_sp/rgear;

% Motor current needed
I_m = T_m/Kmm;

% Input Voltage needed
V_mod = I_m/Kelm;

tau_test = tau_m-2*Kc.*sign(qd_m)./(rsp*rgear*Kmm*Kelm);

% plot results
indices = find(qd_m>0);
indices2 = find(qd_m<0);
figure(fig1);
subplot(2,1,1)
plot((q_m(indices)-data_offset)./pi*180,tau_m(indices),'color',ps.list{i}); hold all;
plot((q_m(indices2)-data_offset)./pi*180,tau_test(indices2),':','color',ps.list{i}); hold all;
plot((q_m(indices)-data_offset)./pi*180,V_mod(indices),'color',ps.list{i});
ylabel('model error +vel');
subplot(2,1,2)
plot((q_m(indices2)-data_offset)./pi*180,tau_m(indices2),'color',ps.list{i}); hold all;
plot((q_m(indices)-data_offset)./pi*180,tau_test(indices),':','color',ps.list{i}); hold all;
plot((q_m(indices2)-data_offset)./pi*180,V_mod(indices2),'color',ps.list{i});
ylabel('model error -vel');
% subplot(2,1,1)
% plot(((q_m(indices))),tau_m(indices),'color',ps.list{i}); hold all;
% plot(((q_m(indices2))),tau_test(indices2),':','color',ps.list{i}); hold all;
% plot(((q_m(indices))),V_mod(indices),'color',ps.list{i});
% ylabel('model error +vel');
% subplot(2,1,2)
% plot(((q_m(indices2))),tau_m(indices2),'color',ps.list{i}); hold all;
% plot(((q_m(indices))),tau_test(indices),':','color',ps.list{i}); hold all;
% plot(((q_m(indices2))),V_mod(indices2),'color',ps.list{i});
% ylabel('model error -vel');

figure(fig2);
% F1= fit(q_m(indices)-data_offset,tau_m(indices)-V_mod(indices),'poly3');
% F2= fit(q_m(indices2)-data_offset,tau_m(indices2)-V_mod(indices2),'poly3');
subplot(2,1,1)
plot((q_m(indices)-data_offset)./pi*180,tau_m(indices)-V_mod(indices),'color',ps.list{i}); hold all;
ylabel('model error +vel');
legend(leg_list2)
subplot(2,1,2)
plot((q_m(indices2)-data_offset)./pi*180,tau_m(indices2)-V_mod(indices2),'color',ps.list{i}); hold all;
ylabel('model error -vel');

% Invest gravity torque
M_joint_meas = tau_m.*(rsp*rgear*Kmm*Kelm).*sin(thF).*lF;

% figure(fig3);
% subplot(2,1,1)
% plot((q_m(indices)-data_offset)./pi*180,M_joint_meas(indices),'color',ps.list{i}); hold all;
% ylabel('Moment [Nm] +vel');
% legend(leg_list2)
% subplot(2,1,2)
% plot((q_m(indices2)-data_offset)./pi*180,M_joint_meas(indices2),'color',ps.list{i}); hold all;
% ylabel('Moment [Nm] -vel');

end
linkaxes(get(fig3,'children'),'x');
set(get(fig3,'CurrentAxes'),'xlim',[40 95]);
params
all_grids_on();
