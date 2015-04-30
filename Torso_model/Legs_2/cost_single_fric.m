function out = cost_single_fric(param,q0,tau_meas,q2,qd0)

    %% estimates
    P1= param(1);   % fric proportional 
%     P2= param(1);
%     P3= param(2);
%     P4 = 2.4724;
    P5 = param(2);  % fric const
    
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
    P2 = l_cm1*m_1 + l_1*m_2 + l_1*m_3 + l_1*m_4;
    P3 = l_cm2*m_2 + l_2*m_3 + l_2*m_4;
    P4 = l_cm3*m_3 +l_3*m_4;
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
    
    
    %% nonlinear functions
    

    
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
    C1 = 1/(l_F1*r_gear1*r_sp);
    f1 = (f2+f3+f4).*C1./sin(thF1).*P1;
    
    % friction
    F_coul = P5;
    
    tau_mod = f1+F_coul;
    
    out = tau_meas-tau_mod;
end