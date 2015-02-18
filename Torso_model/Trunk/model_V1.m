clear all; 
% close all; 
clc;
%-------------------------------------------------------------------------%
% Model: Trunk
%-------------------------------------------------------------------------%

%% get measures from model
run torso_measures_NX.m

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

th_0_min = spindle2_to_angle2(min_spindle2);
th_0_max = spindle2_to_angle2(max_spindle2);
l1 = lJL;
lcm1 = l1/2;
lF = lJK;
th_r = -th_leg+angle0_to_angle1(th_leg);
m1 = 5;
m2 = 10;


% clear parameters
clearvars -except l1 lcm1 lF th_r m1 m2 th_0_min th_0_max g

%%
syms th0 th0d th0dd

% motion equations
M = m1*lcm1^2+m2*l1^2;
H = g*(m1*lcm1+m2*l1)*cos(th0-th_r);

%% compute gravity compensation
xsp_max = angle2_to_spindle2(th_0_max);
xsp_min = angle2_to_spindle2(th_0_min);
xsp_n = linspace(xsp_min,xsp_max,20).';
th0_n = spindle2_to_angle2(xsp_n);

fF = figure; fT = figure; fI = figure; fV = figure;
for m2 = [0 10 20]
    % effective force needed
    F_eff = g*(m1*lcm1+m2*l1)*cos(th0_n-th_r)/lF;

    % applied force needed
    thF = -0.9902.*th0_n+3.152;
    F_appl = sin(pi/2).*F_eff;

    % Spindle torque needed
    l_ls = 0.002;
    rsp = 2*pi/l_ls;
    T_sp = F_appl/rsp;

    % Motor torque needed
    rgear = 13/3;
    T_m = T_sp/rgear;

    % Motor current needed
    Kmm = 29.2e-3;             % Nm/A,     torque constant
    I_m = T_m/Kmm;

    % Input Voltage needed
    Kelm = 10;
    V_m = I_m/Kelm;


    figure(fF)
    plot(xsp_n,F_eff,xsp_n,F_appl); ylabel('Force [N]');hold all;
    figure(fT)
    plot(xsp_n,T_sp,xsp_n,T_m); ylabel('Torque [Nm]');hold all;
    figure(fI)
    plot(xsp_n,I_m); ylabel('Current [A]');hold all;
    figure(fV)
    plot((th0_n-th_r)./pi*180,V_m); ylabel('Voltage [V]'); hold all;
    ylim([-0.2 0.4]);

end
all_grids_on();
