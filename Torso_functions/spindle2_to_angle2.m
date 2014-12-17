function out = spindle2_to_angle2(x)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX.m
    
    out = cosinerule(di(H,J),di(J,K),x,[])+an(G,J,H)+an(K,J,L);
    
end
    
    