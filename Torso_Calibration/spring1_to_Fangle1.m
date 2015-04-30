function out = spring1_to_Fangle1(x)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
    
    out = cosinerule(di(A,F),x,di(A,B),[]);
    
end
    
    