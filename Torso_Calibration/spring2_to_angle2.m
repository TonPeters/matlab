function out = spring2_to_angle2(x)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
    
    out = cosinerule(di(I,J),di(J,K),x,[])+an(G,J,I)+an(K,J,L);
    
end
    
    