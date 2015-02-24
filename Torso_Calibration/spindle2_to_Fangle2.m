function out = spindle2_to_Fangle2(xsp)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
       
    out = cosinerule(di(J,K),xsp,di(H,J),[]);
    
end
    
    