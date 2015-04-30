function r = ratio_dq1_q0_exact(q0)
% GEAR_Q1_Q0, calculates the change of q1 depenent on q0 at location q0.

    syms q0_sym
    q1 = angle0_to_angle1(q0);
    dq1_q0 = sym_partial_derivative(q1,q0);
    
    r = subs(dq1_q0,q0_sym,q0);
end