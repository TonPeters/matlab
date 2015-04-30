function cost = tau_trunk(q2,q0,qd2,m_4)
    
    %% Known Parameters
    l_3 = 0.47;
    l_cm3 = l_3/2;
    l_ls = 0.002;               % lead of lead screw
    r_sp = 2*pi/l_ls;            % gear ratio lead screw
    r_gear2 = 13/3;               % gear ratio gear box
    K_m = 29.2e-3;              % Nm/A,     torque constant
    K_elm = 10;                  % Gain bij Elmo violin
    g = 9.81;                   % gravity constant
    l_F2 = 0.079;
    
     %% Estimation paramters
     
    param = [2.548 0.0290 0.0097];
    
    % mass
    P3 = param(1);

    % Coulomb friction
    Kcoul = param(2);
    
    % Directional dependent friction
    Kdir = param(3);
        
    
    %% Variables
    thF2 = spindle2_to_Fangle2(angle2_to_spindle2(q2));
    q1 = angle0_to_angle1(q0);
    th_r = q1-q0; 
    
    %% equations of motion
    % Moment needed
    M_joint = (g*(P3+m_4.*l_3).*cos(q2-th_r+2/180*pi));

    % Gravity force
    F_g = M_joint./l_F2./sin(thF2);

    % Spindle torque needed
    tau_sp = F_g./(r_sp*r_gear2);

    % Motor torque needed
    tau_m = tau_sp+sign(qd2).*Kcoul+Kdir;
    
    %% Cost
    cost = tau_m-tau_meas;
    
end