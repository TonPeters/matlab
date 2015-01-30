function out = angle0_to_angle1(theta)
% calculate angle0 (angkle) from spindle length

    run torso_measures_NX
    
    % determine distances
    AG = di(A,G); AB = di(A,B); BD = di(B,D); DG = di(D,G);
    % caluclate distance BG
    BG = cosinerule(AB,AG,[],theta+an(B,A,A+[0;-1;0]));
    % caluclate angle AGD by contributions AGB and BGD
    AGB = cosinerule(AG,BG,AB,[]);
    BGD = cosinerule(BG,DG,BD,[]);
    % determine angle DGJ
    DGJ = an(D,G,J);
    
    % angle1 is
    out = DGJ-AGB-BGD;
    
end
    
    