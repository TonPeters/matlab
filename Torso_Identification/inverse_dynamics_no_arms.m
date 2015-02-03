function out = inverse_dynamics_no_arms(qr,qdr,qddr)

    load motion_no_spring_no_arms.mat
    
    Sr = double(subs(S,q,qr));
    Mr = double(subs(M,[q;qd],[qr;qdr]));
    Hr = double(subs(H,[q;qd],[qr;qdr]));
    
    out = inv(Sr)*(Mr*qddr+Hr);  
    
end