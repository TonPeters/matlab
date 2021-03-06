function F_trunk = sim_trunk2_test_gear6(q0,q2,qd,qdd,m_4,M_4,N_trunk_set,F1_trunk_set)
% TRUNK_TORQUE, determine the required torque by the trunk based on the 
%   model of the trunk. Inputs are
%   (q0,q2,m_4,N_leg,F1_leg,N_trunk,F1_trunk):
%   -   q0, joint angle 0 (ankle)
%   -   q2, joint angle 2 (hip)
%   -   dq, velocity directions
%   -   m_4, mass of the arms
%   -   M_4, moment generated by the arms
%   -   N_leg, number of springs in leg
%   -   F1_leg, load of springs in leg
%   -   N_trunk, number of springs in trunk
%   -   F1_trunk, load of springs in trunk

    %% Parameters used
    
    % dimensions
    run torso_measures_NX
    lAG = di(A,G);
    lAE = di(A,E);
    lAF = di(A,F);
    lGJ = di(G,J);
    lJK = di(J,K);
    lJL = di(J,L);
    l_1 = lAG;
    l_2 = lGJ;
    l_3 = lJL;
    l_F2 = lJK;                                      % arm of applied spindle force on joint 2
    l_F1 = lAE;                                      % arm of applied spindle force on joint 1
    l_Fgs2 = lJK;                                      % arm of applied spring force on joint 2
    l_Fgs1 = lAF;                                      % arm of applied spring force on joint 1

    
    % constants
    g = 9.81;
    
    % drive train
    r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
    r_gear2 = 6/1;         % rad/rad,      Gear ratio from spindle to motor 2
    l_ls    = 0.002;        % m,           Lead of the spindle
    r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation
    
    % estimated parameters
    param = [6.704 5.672 0.207 -0.00756 0.0095];
    P1 = param(1);
    P2 = param(2);    
    Kls_1 = param(3);
    Kd_1 = param(4);
    Kc_1 = param(5);
    param = [2.548 0.0290 0.0097];
    P3 = param(1);
    Kc_2 = param(2);
    Kd_2 = param(3);
    q2_gravity_offset = 2/180*pi;   % offset angle of gravity of joint q2

    %% variables used
    
    % other parameters depenant on joint angles
    q1 = angle0_to_angle1(q0);
%     th_F1 = spindle1_to_Fangle1(angle0_to_spindle1(q0)); % angle of force applied by spindle 1
    th_F2 = spindle2_to_Fangle2(angle2_to_spindle2(q2)); % angle of force applied by spindle 2
    dq1_dq0 = ratio_dq1_q0(q0);                          % change of q1 dependant on q0
    
    syms qs0 qs2
    
    %% calculate the spring loads

    B_trunk  = 404e-3+2*30e-3;  % complete extended length m
    A_trunk  = 175e-3;          % spring motion range m
    % F1_trunk = 800;             % force min length
    X_trunk  = 1.35;            % f2/f1
    F2_trunk = F1_trunk_set*X_trunk;% force min length
    K_trunk  = (F2_trunk-F1_trunk_set)/(A_trunk-10e-3); % spring constant (slope N/m)
    FR_trunk = 60;              % direction dependant force
    L0_trunk = B_trunk  + (F1_trunk_set)/K_trunk; % length at zero energy m (not reachable)
    
    %% gravity contribution
    g_trunk = g*(P3+m_4.*l_3).*cos(q2-q1+q0+q2_gravity_offset);

    % Moment of the arms
    g_arms_trunk = M_4;
    
    
    %% spring contribution
    % gas spring force
    dlspr2_dq = sym_partial_derivative(angle2_to_spring2(qs2),qs2);
    dlspr2_dq_num = double(subs(dlspr2_dq,qs2,q2));
    lspr2 = angle2_to_spring2(q2);                                  % length spring trunk
    gs_trunk = -N_trunk_set*K_trunk.*(L0_trunk-lspr2).*dlspr2_dq_num;   % force gas spring trunk

    % direction dependant gas spring force
    thFgs2 = spring2_to_Fangle2(lspr2);
    F_gs2 = -N_trunk_set.*FR_trunk*sin(thFgs2).*l_Fgs2;

    %% potential contribution
    % continuous
%     G_leg = g_leg+gs_leg+g_arms_leg;
    G_trunk = g_trunk+gs_trunk+g_arms_trunk;
    
    % direction dependant contribution
	if qd<0
        G_trunk = G_trunk+F_gs2;
    end

    % motor torques by potential
    tau_G_trunk = G_trunk./(l_F2.*sin(th_F2).*(r_sp*r_gear2));

    %% friction contributions
    tau_fric_trunk = sign(qd).*Kc_2+Kd_2;
    
    %% inertia contributions
    load Inertia_arms_20kg.mat
    
    % inertia by arm mass.
    if m_4~=0
        D_m4_trunk = D_m4*qdd;
        tau_D_arms_trunk = double(subs(D_m4_trunk(2),{'q0','q2'},{q0,q2}));
        if qd<1
            tau_D_arms_trunk = -tau_D_arms_trunk;
        end
        if m_4==10
%             tau_D_arms_leg = tau_D_arms_leg./2;
            tau_D_arms_trunk = tau_D_arms_trunk./2;
        else
            assert(m_4==20,'wrong arm mass m_4, should be 0, 10 or 20');
        end
    else
        tau_D_arms_trunk = zeros(size(q0));
    end

    % inertia contribution by torso mechanism
    D_m123_trunk = D_torso*qdd;
    tau_D_trunk = double(subs(D_m123_trunk(2),{'q0','q2'},{q0,q2}));
    if qd<0
        tau_D_trunk = -tau_D_trunk;
    end

    %% motor torques
%     tau_leg = (tau_fric_leg+tau_G_leg+tau_D_arms_leg+tau_D_leg);
    tau_trunk = (tau_fric_trunk+tau_G_trunk+tau_D_arms_trunk+tau_D_trunk);
    
%     F_leg = tau_leg.*(r_sp*r_gear1);
    F_trunk = tau_trunk.*(r_sp*r_gear2);
    
end
    
    
    
    