function tau_m = tau_legs(q0,q2,qd0,m_4)

    %% estimates
    param = [6.704 5.672 0.207 -0.00756 0.0095];
    P1 = param(1);
    P2 = param(2);    
    Kls = param(3);
    Kdir = param(4);
    Kcoul = param(5);

    P3 = 2.548;
    
    %%
%     l_1 = 0.39;
%     l_2 = 0.4115;
%     l_3 = 0.47;
%     m_4 = 0;
%     l_cm1 = l_1/2;
%     l_cm2 = l_2/2;
%     l_cm3 = l_3/2;
%     m_3 = P3/l_cm3;
%     P1 = l_cm1*m_1 + l_1*m_2 + l_1*m_3 + l_1*m_4;
%     P2 = l_cm2*m_2 + l_2*m_3 + l_2*m_4;
    %% known parameters
    % gravity constant
    g = 9.81;                   
    
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
    
    % Gravity torque needed
    T_leg = g.*(cos(q0).*(P1+l_1*m_4)+...             
        (dq1_q0-1).*cos(q1-q0).*(P2+l_2*m_4)+...     
        (1-dq1_q0).*cos(q2-q1+q0+q2_gravity_offset).*(P3+l_3*m_4));
    
    % motor torque by gravity
    tau_g = T_leg./sin(thF1)./(l_F1*r_gear1*r_sp);
    
    % motor torque by friction
    tau_fric = sign(qd0).*Kcoul+Kdir;
    
    % motor torque
    tau_m = tau_g.*(1+sign(qd0)*Kls)+tau_fric;
    
end