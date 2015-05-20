function out = plot_torso_base()


    plotsettings
    run torso_measures_NX
    l1 = di(A,G);
    l2 = di(G,J);       % length from knee to hip
    lcm2 = l2-di(D,J)/2;    % length rotation point to center of mass (half of leg length)
    l3 = di(J,L);       % length from hip to shoulder
    lcm3 = di(J,L)/2;
    lFsp1 = di(A,E);
    lFsp2 = di(J,K);
    shift = A(2);
    
    
    zshift =  0.025;
    % plot base limits
    out = plot(-[BP_front(2) BP_back(2)],[0 0],'-+','color',ps.orange,'linewidth',1); hold all;
    
end

