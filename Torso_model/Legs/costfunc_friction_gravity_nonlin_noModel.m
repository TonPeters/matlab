function cost = costfunc_friction_gravity_nonlin_noModel(param,q,qd,V_meas,q2)

    % joints
    q0 = q;
    q1 = angle0_to_angle1(q0);
    
    % Estimation paramters
%     Fc = param(1);
    P1 = param(1);
    P2 = param(2);
    m3_lcm3 = param(3);
    
    
    % Parameters   
    g = 9.81;                   % gravity constant
    m3_lcm3 = 2.7955;           % m3*lcm3
    
%     % Joint moments needed
%     M_0 = lcm1.*cos(q0).*m1*g+(l1.*cos(q0)-lcm2.*cos(q1-q0)).*m2*g+...
%         (l1.*cos(q0)-l2.*cos(q1-q0)+lcm3.*cos(q2-q1+q0)).*m3*g;
%     M_1 = lcm2.*cos(q1-q0).*m2*g+(l2.*cos(q1-q0)-lcm3.*cos(q2-q1+q0)).*m3*g;
%     
%     % leg moment needed
%     M_leg = M_0+r_q1_q0.*M_1;
    
    % Joint moments needed (refactored)
    r_q1_q0 =  1.8546;
    M_leg = g.*(cos(q0).*P1+...             % P1 = lcm1*m1+l1*m2+l1*m3
        (r_q1_q0-1).*cos(q1-q0).*P2+...     % P2 = lcm2*m2+l2*m3
        (1-r_q1_q0)*cos(q2-q1+q0)*m3_lcm3);
        
    % effective force needed
    lF1 = 0.3509;
    F_eff = M_leg./lF1;
    
    % applied force needed
    thF1 = spindle1_to_Fangle1(angle0_to_spindle1(q0));
    F_appl = F_eff./sin(thF1);
    
    % add coulomb friction
    F_appl_fric = F_appl;
    F_appl_fric = F_appl.*(1+sign(qd).*Fc);

    % Spindle torque needed
    l_ls = 0.002;
    rsp = 2*pi/l_ls;
    T_sp = F_appl_fric/rsp;

    % Motor torque needed
    rgear = 5/2;
    T_m = T_sp/rgear;

    % Motor current needed
    Kmm = 29.2e-3;             % Nm/A,     torque constant
    I_m = T_m/Kmm;

    % Input Voltage needed
    Kelm = 10;
    V_m = I_m/Kelm;
    
    cost = V_meas-V_m;
    
end