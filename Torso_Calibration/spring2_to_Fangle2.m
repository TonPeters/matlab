function out = spring2_to_Fangle2(x)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
       
    out = cosinerule(di(J,K),x,di(I,J),[]);
    
end
    
    