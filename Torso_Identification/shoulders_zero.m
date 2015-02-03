function q2 = shoulders_zero(q0)

    run torso_measures_NX
    q1 = angle0_to_angle1(q0);
    q2 = acos((A(2)-di(A,G)*cos(q0)+di(G,J)*cos(q1-q0))/di(J,L))-q0+q1;
    
end