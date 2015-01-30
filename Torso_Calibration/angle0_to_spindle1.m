function out = angle0_to_spindle1(theta)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
    
    out = cosinerule(di(A,E),di(A,C),[],theta+an(C,A,A+[0;-1;0]));
    
end
    
    