function out = spring1_to_angle0(x)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
    
    out = cosinerule(di(A,F),di(A,B),x,[])-an(B,A,A+[0;-1;0]);
    
end
    
    