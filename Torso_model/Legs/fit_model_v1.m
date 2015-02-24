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
g = 9.81;

th_2_min = spindle2_to_angle2(min_spindle2);
th_2_max = spindle2_to_angle2(max_spindle2);
th_0_min = spindle1_to_angle0(min_spindle1);
th_0_max = spindle1_to_angle0(max_spindle1);
l3 = lJL;
% lcm3 = l1*0.7;
lF2 = lJK;
lF1 = lAE;
l1 = lAG;
l2 = lGJ;
l3 = lJL;

% estimates
lcm1 = l1/2;
lcm2 = l2/2;
lcm3 = l3/2;
m1 = 7;
m2 = 7;
% m3 = m3_lcm3/lcm3;
m3 = 7;

m3_lcm3 = 2.7955;
m3_lcm3 = 2.5;


% clear parameters
clearvars -except g m3_lcm3 l1 l2 l3 lcm1 lcm2 lcm3 m1 m2 m3 lF1 th_0_min th_0_max m3_lcm3
plotsettings

%% fit data
xsp0_max = angle0_to_spindle1(th_0_max);
xsp0_min = angle0_to_spindle1(th_0_min);
xsp0_n = linspace(xsp0_min,xsp0_max,20).';
q0 = spindle1_to_angle0(xsp0_n);
qd0 = ones(size(q0));
q0 = [q0];%;q0];
qd0 = [qd0];%;-qd0];
q1 = angle0_to_angle1(q0);

% Gear ratio joint 1 depending on joint 0
% r_q1_q0 = diff(angle0_to_angle1(q0));
% r_q1_q0 = [r_q1_q0;r_q1_q0(end)]./q0;
r_q1_q0 =  1.8546;

%% iterate over data sets
joint = 1;
load_list = {'10','14','16'};
q2_list = [1.01, 1.4, 1.6];
Fc_list = [228.3295;  252.4930;  263.6036];
params = zeros(3,4);

figc = figure;
for i = 1:3
    %% load data
    load(['../../Legs_Identification/data_friction/m0kg_vel004_trunk',load_list{i},'.mat']);
    q0_meas = spindle1_to_angle0(enc(:,joint));
    qd0_meas = ones(size(q0_meas));
    qd0_meas(indices.n1) = -ones(length(indices.n1),1);
    qd0_meas(indices.n2) = -ones(length(indices.n2),1);
    tau_meas = u(:,1);
    
    %% data to fit
    q0_m = [q0_meas(indices.p1);q0_meas(indices.p2);q0_meas(indices.n1);q0_meas(indices.n2)];
    qd0_m = [qd0_meas(indices.p1);qd0_meas(indices.p2);qd0_meas(indices.n1);qd0_meas(indices.n2)];
    tau_m = [tau_meas(indices.p1);tau_meas(indices.p2);tau_meas(indices.n1);tau_meas(indices.n2)];

    %% optimization fit
    param0 =    [20;    lcm1*m1+l1*m2+l1*m3;    lcm2*m2+l2*m3; m3_lcm3];
    lb =        [0;    0; 0; 0];
    ub =        [1000;  100;        100; 100];
    
    
    % fminunc
    % [param,resnorm,exitflag] = fminunc(@(param)costfunc_friction_gravity(param,q0_meas(indices.p1),qd0_meas(indices.p1),tau_m(indices.p1),q2_list(i)),param0)

    % lsqnonlin
    opts = optimset('lsqnonlin');
    opts = optimset(opts,'tolfun',1E-16,'display','off','tolx',1E-16);
%     [param,resnorm,residual,exitflag] = lsqnonlin(@(param)costfunc_friction_gravity_nonlin(param,q0_m,qd0_m,tau_m,q2_list(i)),param0,lb,ub,opts);
%     param
%     exitflag
    
    problem = createOptimProblem('lsqnonlin','objective',...
        @(param)costfunc_friction_gravity_nonlin_noModel(param,q0_m,qd0_m,tau_m,q2_list(i)),...
        'x0',param0,'lb',lb,'ub',ub,'options',opts);
    ms = MultiStart('Display','off');
    n_startpoints = 2;
    [param,fval,exitflag] = run(ms,problem,n_startpoints)
    
    
    % Estimation paramters
    params(i,:) = param;
    Fc = param(1);
    P1 = param(2);
    P2 = param(3);
    m3_lcm3 = param(4);
    
    % parameters positive velocity
%     0.0000    0.0000   15.5322    1.8961
%     0.0000    0.0000   15.8084    1.8670
%     0.0000    0.0000   15.8709    1.5477
%     params = [0.0000    0.0000   15.7372    1.7702];

% parameters positive velocity
%     0.0000    0.0000   15.5322    1.8961
%     0.0000    0.0000   15.8084    1.8670
%     0.0000    0.0000   15.8709    1.5477
%     params = [0.0000    0.0000   15.7372    1.7702];

    %% compute gravity compensation
    % xsp2 = 0.36;
    % th2 = spindle2_to_angle2(xsp2);




    % fF = figure; fT = figure; fI = figure; fV = figure; 

    % fM = figure;

    q2 = q2_list(i);
%     % Joint moments needed
%     M_0 = lcm1.*cos(q0).*m1*g+(l1.*cos(q0)-lcm2.*cos(q1-q0)).*m2*g+...
%         (l1.*cos(q0)-l2.*cos(q1-q0)+lcm3.*cos(q2-q1+q0)).*m3*g;
%     M_1 = lcm2.*cos(q1-q0).*m2*g+(l2.*cos(q1-q0)-lcm3.*cos(q2-q1+q0)).*m3*g;
%     M_2 = lcm3.*cos(q2-q1+q0)*m3*g;
%     
%     % leg moment needed
%     M_leg = M_0+r_q1_q0.*M_1;
    
    % Joint moments needed (refactored)
    M_leg = g.*(cos(q0).*P1+...             % P1 = lcm1*m1+l1*m2+l1*m3
        (r_q1_q0-1).*cos(q1-q0).*P2+...     % P2 = lcm2*m2+l2*m3
        (1-r_q1_q0)*cos(q2-q1+q0)*m3_lcm3);
    
    
    % effective force needed
    F_eff = M_leg./lF1;
    
    % applied force needed
    thF1 = spindle1_to_Fangle1(angle0_to_spindle1(q0));
    F_appl = F_eff./sin(thF1);

    % add coulomb friction
%     F_appl_fric = F_appl+Fc;
    F_appl_fric = F_appl.*(1+sign(qd0).*Fc);
    
    % Spindle torque needed
    l_ls = 0.002;
    rsp = 2*pi/l_ls;
    T_sp = F_appl_fric/rsp;

    % Motor torque needed
    rgear = 5/2;
    T_m = T_sp/rgear;

    % Motor current needed
    Kmm = 29.2e-3;             % Nm/A,     torque constant
    I_m = T_m/Kmm;

    % Input Voltage needed
    Kelm = 10;
    V_m = I_m/Kelm;
    V_mi{i} = V_m;


%     figure(fF)
%     plot(q0./pi*180,M_0,':','color',ps.list{i}); hold all;
%     plot(q0./pi*180,M_1.*r_q1_q0,'--','color',ps.list{i}); hold all;
%     plot(q0./pi*180,M_leg,'-','color',ps.list{i}); hold all;
%     ylabel('Moment');
%     figure(fT)
%     plot(q0./pi*180,T_sp,q0./pi*180,T_m); ylabel('Torque [Nm]');hold all;
%     figure(fI)
%     plot(q0./pi*180,I_m); ylabel('Current [A]');hold all;
%     figure(fV)
%     plot((q0)./pi*180,V_m,'*'); ylabel('Voltage [V]'); hold all;
%     ylim([-0.2 0.4]);
    
%     load(['../../Legs_Identification/data_friction/m0kg_vel004_trunk',load_list{i},'.mat']);
    figure(figc);
    plot((q0)./pi*180,V_m,'color',ps.list{i+2}); hold all; 
    plot(q0_meas./pi*180,tau_meas,':','color',ps.list{i+2});

end
figure(figc); ylim([0 0.4]);
% close(fF,fT,fI);
params
%% save
% legend('meas 0kg','meas 10kg','meas 20kg','mod 0kg','mod 10kg','mod 20kg','Location','Eastoutside')
% save_report(gcf,'../../../../Report/Images/Model','trunk_gravity','customleg',[10 6]);

all_grids_on();
