function cost = cost_nonlin(param,q2,tau_meas,m_2,th_r,qd2)
    
    %% Known Parameters
    l_3 = 0.47;
    l_cm3 = l_3/2;
    m_3 = 10;
    l_ls = 0.002;               % lead of lead screw
    r_sp = 2*pi/l_ls;            % gear ratio lead screw
    r_gear2 = 13/3;               % gear ratio gear box
    K_m = 29.2e-3;              % Nm/A,     torque constant
    K_elm = 10;                  % Gain bij Elmo violin
    g = 9.81;                   % gravity constant
    l_F2 = 0.079;
    
     %% Estimation paramters
    P1 = param(1);
    P2 = param(2);
    P3 = param(3);
%     P4 = param(4);
    
    
    %% Variables
    thF2 = spindle2_to_Fangle2(angle2_to_spindle2(q2));
    
    %% mass shoulders
    % Moment needed
    M_joint = (g*m_2*l_3).*cos(q2-th_r+2/180*pi);

    % Gravity force
    F_g = M_joint./l_F2./sin(thF2);

    % Spindle torque needed
    tau_g = F_g./(r_sp*r_gear2);
    
    %% equations of motion
    % Moment needed
    f1 = cos(q2-th_r+2/180*pi);

    % Gravity force
    f2 = f1./sin(thF2);

    % Motor torque needed
    tau_m = (P1.*f2+tau_g).*P3+P2;
    
    %% Cost
    cost = tau_m-tau_meas;
    
end