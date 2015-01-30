function out = angle2_to_spindle2(theta)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
    
    out = cosinerule(di(H,J),di(J,K),[],theta-an(G,J,H)-an(K,J,L));
    
end
    
    