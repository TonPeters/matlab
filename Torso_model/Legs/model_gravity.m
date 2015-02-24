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
% m1_lcm1 = 2.63;
m3_lcm3 = 2.2819;
l1 = lAG;
l2 = lGJ;
l3 = lJL;

% estimates
lcm1 = l1/2;
lcm2 = l2/2;
lcm3 = l3/2;
m1 = 5;
m2 = 5;
% m3 = m3_lcm3/lcm3;
m3 = 7;



% clear parameters
clearvars -except g m3_lcm3 l1 l2 l3 lcm1 lcm2 lcm3 m1 m2 m3 lF1 th_0_min th_0_max
plotsettings

%% compute gravity compensation
% xsp2 = 0.36;
% th2 = spindle2_to_angle2(xsp2);

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
    
i=1;
% fF = figure; fT = figure; fI = figure; fV = figure; 
figc = figure;
fM = figure; scr l;
fM2 = figure; scr r;
load_list = {'10','14','16'};
q2_list = [1.01, 1.8, 2.3];
for i = 1:length(q2_list) 
    q2 = q2_list(i);
    % Joint moments needed
    M_0 = lcm1.*cos(q0).*m1*g+(l1.*cos(q0)-lcm2.*cos(q1-q0)).*m2*g+...
        (l1.*cos(q0)-l2.*cos(q1-q0)+lcm3.*cos(q2-q1+q0)).*m3*g;
    M_1 = lcm2.*cos(q1-q0).*m2*g+(l2.*cos(q1-q0)-lcm3.*cos(q2-q1+q0)).*m3*g;
    M_2 = lcm3.*cos(q2-q1+q0)*m3*g;
    
%     subplot(3,1,1);
%     plot(q0./pi*180,(l1.*cos(q0)-l2.*cos(q1-q0)+lcm3.*cos(q2-q1+q0)).*1,...
%         q0./pi*180,(l1.*cos(q0)-lcm2.*cos(q1-q0)).*1,q0./pi*180,lcm1.*cos(q0).*1,...
%         q0./pi*180,M_0);
%     legend('m3','m2','m1','sum')
%     subplot(3,1,2);
%     plot(q0./pi*180,(l2.*cos(q1-q0)-lcm3.*cos(q2-q1+q0)).*1,q0./pi*180,lcm2.*cos(q1-q0).*1,...
%         q0./pi*180,M_1);
%     legend('m3','m2','sum');
%     subplot(3,1,3);
%     plot(q0./pi*180,lcm3.*cos(q2-q1+q0));
%     legend('m3');
    
    
    % leg moment needed
    M_leg = M_0+r_q1_q0.*M_1;
    
    
    
    
    % effective force needed
    F_eff = M_leg./lF1;
    
    % applied force needed
    thF1 = spindle1_to_Fangle1(angle0_to_spindle1(q0));
    F_appl = F_eff./sin(thF1);

    % Spindle torque needed
    l_ls = 0.002;
    rsp = 2*pi/l_ls;
    T_sp = F_appl/rsp;

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

    
    figure(fM);
    subplot(2,1,1);
    plot(q0./pi*180,M_0,'--','color',ps.list{i}); hold all;
    plot(q0./pi*180,M_1.*r_q1_q0,':','color',ps.list{i});
    plot(q0./pi*180,M_leg,'color',ps.list{i});
    ylabel('Moment needed');
    subplot(2,1,2);
    plot(q0./pi*180,M_0./lF1./sin(thF1),'--','color',ps.list{i}); hold all;
    plot(q0./pi*180,M_1.*r_q1_q0./lF1./sin(thF1),':','color',ps.list{i});
    plot(q0./pi*180,M_leg./lF1./sin(thF1),'color',ps.list{i});
    ylabel('Force needed'); xlabel('angle0 deg.');
    
    M1 = g.*(cos(q0).*(lcm1*m1+l1*m2+l1*m3));            % P1 = lcm1*m1+l1*m2+l1*m3
    M2 = g.*((r_q1_q0-1).*cos(q1-q0).*(lcm2*m2+l2*m3));     % P2 = lcm2*m2+l2*m3
    M3 = g.*((1-r_q1_q0)*cos(q2-q1+q0)*m3_lcm3);
    M_leg2 = M1+M2+M3;
    
    figure(fM2);
    subplot(2,1,1);
    plot(q0./pi*180,M1,'--','color',ps.list{i}); hold all;
    plot(q0./pi*180,M2,'*','color',ps.list{i});
    plot(q0./pi*180,M3,'-.','color',ps.list{i});
    plot(q0./pi*180,M_leg2,'color',ps.list{i});
    ylabel('Moment needed');
    subplot(2,1,2);
    plot(q0./pi*180,M1./lF1./sin(thF1),'--','color',ps.list{i}); hold all;
    plot(q0./pi*180,M2./lF1./sin(thF1),'*','color',ps.list{i});
    plot(q0./pi*180,M3./lF1./sin(thF1),'-.','color',ps.list{i});
    plot(q0./pi*180,M_leg2./lF1./sin(thF1),'color',ps.list{i});
    ylabel('Force needed'); xlabel('angle0 deg.');

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
    
    load(['../../Legs_Identification/data_friction/m0kg_vel004_trunk',load_list{i},'.mat']);
    figure(figc);
    plot((q0)./pi*180,V_m,'color',ps.list{i+2}); hold all; 
    plot(spindle1_to_angle0(enc(:,1))./pi*180,u(:,1),':','color',ps.list{i+2});

end
% close(fF,fT,fI);
%% measurements



% legend('meas 0kg','meas 10kg','meas 20kg','mod 0kg','mod 10kg','mod 20kg','Location','Eastoutside')
% save_report(gcf,'../../../../Report/Images/Model','trunk_gravity','customleg',[10 6]);

all_grids_on();
