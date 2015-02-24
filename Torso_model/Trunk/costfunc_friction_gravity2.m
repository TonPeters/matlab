function cost = costfunc_friction_gravity2(param,q,qd,V_meas,m2,th_r)

    % Estimation paramters
    m1_lcm1 = param(1);
    Kc = param(2);
    Kd = param(3);
    
    % Parameters
%     m2 = 0;
    l1 = 0.47;
    l_ls = 0.002;               % lead of lead screw
    rsp = 2*pi/l_ls;            % gear ratio lead screw
    rgear = 13/3;               % gear ratio gear box
    Kmm = 29.2e-3;              % Nm/A,     torque constant
    Kelm = 10;                  % Gain bij Elmo violin
    g = 9.81;                   % gravity constant
%     th_leg = 0.56;              % joint angle leg
%     th_r = -th_leg+angle0_to_angle1(th_leg); % offset angle joint trunk
    lF = 0.079;
    
    % Moment needed
    M_joint = (g*(m1_lcm1+m2.*l1).*cos(q-th_r));
    
    % Effective force needed
    F_eff = M_joint./lF;
    
    % Friction term
%     F_eff = Feff+Kj*sign(qd)

    % applied force needed
%     thF = -0.9902.*q+3.152;     % Angle force applied
    thF = spindle2_to_Fangle2(angle2_to_spindle2(q));
    F_appl = F_eff./sin(thF);
    
    % add Friction force
    Ffr = Kc.*sign(qd)+Kd;       
    F_appl_fric = F_appl+Ffr;

    % Spindle torque needed
    T_sp = F_appl_fric/rsp;

    % Motor torque needed
    T_m = T_sp/rgear;

    % Motor current needed
    I_m = T_m/Kmm;

    % Input Voltage needed
    V_mod = I_m/Kelm;
    
    cost = norm(V_meas-V_mod);
    
end