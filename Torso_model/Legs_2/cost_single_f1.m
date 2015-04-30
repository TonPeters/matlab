function out = cost_single_f1(param,q0,tau_meas,q2)

    %% estimates
    P1= param(1);
    
    
    %% known parameters
    % measures
    l_F1 = 0.3509;           % m             Length at which force is applied
    % drive train
    K_m     = 29.2e-3;      % Nm/A,         Motor torque constant
    K_elm   = 10;           % A/V_input,    Gain from input Voltage to Current     
    r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
    r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
    l_ls    = 0.002;        % mm,           Lead of the spindle
    r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation
    
    %% nonlinear functions
    % applied force angle
    thF1 = spindle1_to_Fangle1(angle0_to_spindle1(q0));
    C1 = 1/(l_F1*r_gear1*r_sp);
    f1 = C1./sin(thF1)*P1;

    
%     % angle q0
% %     C2 = g*(P1+l_1*m4);
%     f2 = cos(q0);
% 
%     % angle q1
% %     C3 = g*(P2+l_2*m4);
%     f3 = (dq1_q0-1).*cos(q1-q0);
% 
%     % angle q2
% %     C4 = g*(P3+l_3*m4);
%     f4 = (1-dq1_q0).*cos(q2-q1+q0+q2_gravity_offset);
    
    
    tau_mod = f1;
    
    out = tau_meas-tau_mod;
end