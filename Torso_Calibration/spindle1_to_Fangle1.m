function out = spindle1_to_Fangle1(x)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
    
    out = cosinerule(di(A,E),x,di(A,C),[]);
    
end
    
    