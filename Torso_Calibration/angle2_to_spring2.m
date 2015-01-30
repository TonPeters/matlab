function out = angle2_to_spring2(theta)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
    
    out = cosinerule(di(I,J),di(J,K),[],theta-an(G,J,I)-an(K,J,L));
    
end
    
    