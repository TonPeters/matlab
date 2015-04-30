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


fig1 = figure; scr r;
    subplot(2,1,1);
for meas = measurements
    for set = sets
        plot(sdata{meas}.q0_m{set},sdata{meas}.tau_m{set},'color',ps.list_div{meas,1}); hold all;
        plot(sdata{meas}.q0_s{set},sdata{meas}.tau_f{set},'--','color',ps.list_div{meas,2});


    end
    title(['q2 ',num2str(sdata{meas}.q2)])
end


%% model calculation
q0 = q0_est;
q2 = sdata{meas}.q2;
qd0 = 0.004;

%% model calculation

    
    

    %% known parameters

    g = 9.81;                   % gravity constant

    % measures
    l_F1 = 0.3509;           % m             Length at which force is applied
    q2_gravity_offset = 2/180*pi;   % gravity offset on joint q2

    % drive train
    K_m     = 29.2e-3;      % Nm/A,         Motor torque constant
    K_elm   = 10;           % A/V_input,    Gain from input Voltage to Current     
    r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
    r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
    l_ls    = 0.002;        % mm,           Lead of the spindle
    r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation

    %% parameters depending on q0
    dq1_q0 = ratio_dq1_q0(q0);        % gear ratio joint 1 to joint 2
    thF1 = spindle1_to_Fangle1(angle0_to_spindle1(q0)); % angle force applied
    q1 = angle0_to_angle1(q0);

    %% parameters
    l_1 = 0.39;
    l_2 = 0.4115;
    l_3 = 0.47;
    m_1 = 3.5;
    m_2 = 5;
    m_3 = 10;
    m_4 = 0;
    l_cm1 = l_1/2;
    l_cm2 = l_2/2;
    l_cm3 = l_3/2;
    i=1;
    %% estimates
    m_1 = 3.5;
    m_2 = 5;
    m_3 = 10.5
    P1 = 1;         % gain
    P5 = -0.02;         % coul
    Kdir = -0.0096;
    
    P2 = l_cm1*m_1 + l_1*m_2 + l_1*m_3 + l_1*m_4;
    P3 = l_cm2*m_2 + l_2*m_3 + l_2*m_4;
    P4 = l_cm3*m_3 +l_3*m_4;
    
    % nonlinear functions
    C1 = 1/(l_F1*r_gear1*r_sp);
    
    fm1 = m_1*g.*(l_cm1*cos(q0)).*C1./sin(thF1);
    fm2 = m_2*g.*(l_1*cos(q0)+l_cm2.*cos(q1-q0).*(dq1_q0-1)).*C1./sin(thF1);
    fm3 = m_3*g.*(l_1*cos(q0)+l_2.*cos(q1-q0).*(dq1_q0-1)+l_cm3.*cos(q2-q1+q0).*(1-dq1_q0)).*C1./sin(thF1);
    
    
    %     % angle q0
    % %     C2 = g*(P1+l_1*m4);
    f2 = g.*cos(q0).*P2;
    % 
    %     % angle q1
    % %     C3 = g*(P2+l_2*m4);
    f3 = g.*(dq1_q0-1).*cos(q1-q0).*P3;
    % 

    % angle q2
    %     C4 = g*(P3+l_3*m4);
    f4 = g.*(1-dq1_q0).*cos(q2-q1+q0+q2_gravity_offset).*P4;

    % applied force angle
    f1 = (f2+f3+f4).*C1./sin(thF1);

    % friction
    F_coul = P5*sign(qd0);

    tau_mod_p = (f1+F_coul).*P1+Kdir;
    tau_mod_n = (f1-F_coul).*P1+Kdir;



    % show results



    figure(fig1);
    subplot(2,1,1);
    plot(q0_est,tau_mod_p,'color',ps.list3{i});
    plot(q0_est,tau_mod_n,'color',ps.list3{i});
    subplot(2,1,2);
    plot(q0_est,fm1,q0_est,fm2,q0_est,fm3)
    i = i+1;





all_grids_on();