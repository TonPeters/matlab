clear all; 
close all; 
% clc;
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
lcm1 = l1*0.7;
lF = lJK;
th_r = -th_leg+angle0_to_angle1(th_leg);
m1 = 8;
m2 = 10;
m1_lcm1 = 2.2819;
fric_c = 393.8;
fric_dir = 128.6;

% clear parameters
clearvars -except l1 lF th_r m2 th_0_min th_0_max g m1_lcm1 fric_c fric_dir th_leg
plotsettings
%%
syms th0 th0d th0dd

% motion equations
% M = m1*lcm1^2+m2*l1^2;
% H = g*(m1*lcm1+m2*l1)*cos(th0-th_r);

%% compute gravity compensation
vel = 0.004;
xsp_max = angle2_to_spindle2(th_0_max);
xsp_min = angle2_to_spindle2(th_0_min);
xsp_n = linspace(xsp_min,xsp_max,5).';
th0_n = spindle2_to_angle2(xsp_n);
th0d_n = ones(size(th0_n));
th0_n = [th0_n;th0_n];
th0d_n = [th0d_n;-th0d_n];

str1 = sprintf('spindle1 = %5.5f',angle0_to_spindle1(th_leg));
disp(str1)
format_str = 'spindle2 = %5.5f, vel = %2.1f, ffw2 = %5.5f'; 
i=1;

syms m2 xsp_n th0d_n th_r
th0_n = spindle2_to_angle2(xsp_n);
% fF = figure; fT = figure; fI = figure; fV = figure;
% for m2 = [0]
    % effective force needed
    F_eff = (g*(m1_lcm1+m2*l1).*cos(th0_n-th_r))./lF;
    C1 = (m1_lcm1+m2*l1)*(g)/lF;

    % applied force needed
%     thF = -0.9902.*th0_n+3.152;
%     thF = spindle2_to_Fangle2(angle2_to_spindle2(th0_n));
    thF = spindle2_to_Fangle2(xsp_n);
%     thF = pi/2;
    F_appl = F_eff./sin(thF);
    
    % Add spindle friction term
    Ffric = fric_c*sign(th0d_n)+fric_dir;
    F_appl_fric = F_appl+Ffric;

    % Spindle torque needed
    l_ls = 0.002;
    rsp = 2*pi/l_ls;
    T_sp = F_appl_fric/rsp;

    % Motor torque needed
    rgear = 13/3;
    T_m = T_sp/rgear;

    % Motor current needed
    Kmm = 29.2e-3;             % Nm/A,     torque constant
    I_m = T_m/Kmm;

    % Input Voltage needed
    Kelm = 10;
    V_m = I_m/Kelm;
%     V_mi{i} = V_m;
    i = i+1;
    
    C_F_to_V = 1/rsp/rgear/Kmm/Kelm;
    
    


%     figure(fF)
%     plot(th0_n,F_eff,'*',th0_n,F_appl,'*'); ylabel('Force [N]');hold all;
%     figure(fT)
%     plot(th0_n,T_sp,th0_n,T_m); ylabel('Torque [Nm]');hold all;
%     figure(fI)
%     plot(th0_n,I_m); ylabel('Current [A]');hold all;
%     figure(fV)
%     plot((th0_n-th_r)./pi*180,V_m,'linewidth',ps.linewidth); ylabel('Voltage [V]'); hold all;
%     ylim([-0.2 0.4]);

% end
% for i=1:1:length(th0_n)
%     str2 = sprintf(format_str,angle2_to_spindle2(th0_n(i)),th0d_n(i),V_m(i));
%     disp(str2);
% end
    

% close(fF,fT,fI);

all_grids_on();
