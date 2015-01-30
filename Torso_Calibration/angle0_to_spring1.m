function out = angle0_to_spring1(theta)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
    
    out = cosinerule(di(A,F),di(A,B),[],theta+an(B,A,A+[0;-1;0]));
    
end
    
    