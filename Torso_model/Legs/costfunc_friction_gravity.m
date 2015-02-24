function cost = costfunc_friction_gravity(param,q,qd,V_meas,q2)

    % joints
    q0 = q;
    q1 = angle0_to_angle1(q0);
    
    % Estimation paramters
    m1 = param(1);
    m2 = param(2);
    m3 = param(3);
    lcm1 = param(4);
    lcm2 = param(5);
    lcm3 = param(6);
    Fc = param(7);
    
    % Parameters
    l1 = 0.39;
    l2 = 0.4115;
    l3 = 0.47;
    lF1 = 0.3509;
    r_q1_q0 =  1.8546;
    
    g = 9.81;                   % gravity constant
    
    % Joint moments needed
    M_0 = lcm1.*cos(q0).*m1*g+(l1.*cos(q0)-lcm2.*cos(q1-q0)).*m2*g+...
        (l1.*cos(q0)-l2.*cos(q1-q0)+lcm3.*cos(q2-q1+q0)).*m3*g;
    M_1 = lcm2.*cos(q1-q0).*m2*g+(l2.*cos(q1-q0)-lcm3.*cos(q2-q1+q0)).*m3*g;
    
    % leg moment needed
    M_leg = M_0+r_q1_q0.*M_1;
    
    % effective force needed
    F_eff = M_leg./lF1;
    
    % applied force needed
    thF1 = spindle1_to_Fangle1(angle0_to_spindle1(q0));
    F_appl = F_eff./sin(thF1);
    
    % add coulomb friction
    F_appl = F_appl+Fc;

    % Spindle torque needed
    l_ls = 0.002;
    rsp = 2*pi/l_ls;
    T_sp = F_appl/rsp;

    % Motor torque needed
    rgear = 5/2;
    T_m = T_sp/rgear;

    % Motor current needed
    Kmm = 29.2e-3;             % Nm/A,     torque constant
    I_m = T_m/Kmm;

    % Input Voltage needed
    Kelm = 10;
    V_m = I_m/Kelm;
    
    cost = norm(V_meas-V_m);
    
end