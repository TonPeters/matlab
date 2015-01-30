function out = spindle1_to_angle0(x)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
    
    out = cosinerule(di(A,E),di(A,C),x,[])-an(C,A,A+[0;-1;0]);
    
end
    
    