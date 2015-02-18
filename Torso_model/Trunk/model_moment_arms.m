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

% model parameters
th_0_min = spindle2_to_angle2(min_spindle2);
th_0_max = spindle2_to_angle2(max_spindle2);
l1 = lJL;
lF = lJK;
th_r = -th_leg+angle0_to_angle1(th_leg);
l_ls = 0.002;
rsp = 2*pi/l_ls;            % gear ratio spindle (length to radians)
rgear = 13/3;               % gear ratio gearbox (joint to motor)
Km = 29.2e-3;              % Nm/A,     torque constant (input current to motor moment)
Kelm = 10;                  % Elmo input conversion (input Volt to current)


% estimated paramters
r_sp2_th2 = 0.07549;        % gear ratio spindle joint angle (radians to meters)
lcm1 = l1/2;
m1 = 8;
m2 = 0;
m1_lcm1 = 2.2819;
fric_c = 393.8;
fric_dir = 128.6;

% clear parameters
clearvars -except l1 lF th_r m2 th_0_min th_0_max g m1_lcm1 fric_c fric_dir rsp rgear Km Kelm r_sp2_th2
plotsettings
%% calculate input voltage needed

figure; scr rt;
% loop over masses
for m2 = [0 10 20]
    % settings
    xspdd = 0.024;              % m/s2
    th0dd = xspdd/r_sp2_th2;    % rad/s2

    % variables
    q = linspace(th_0_min,th_0_max,20).';
    xsp = angle2_to_spindle2(q);
    thF = spindle2_to_Fangle2(xsp);

    % joint moment needed
    M_joint = l1^2*m2*th0dd;
    M_joint/lF;

    % input voltage needed
    V_in = M_joint/(lF*rsp*rgear*Km*Kelm)./sin(thF);

    %% plot results
    plot(q./pi*180,V_in); grid on; xlabel('joint angle [deg]'); ylabel('input Voltage'); hold all;
    
end






