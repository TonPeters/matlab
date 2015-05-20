function [out1,out2,varargout] = plot_torso_end(q0,q2,varargin)

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
    
    q1 = angle0_to_angle1(q0);
    
    % positions of the joints
    x_q0 = [0 0];
    x_q1 = [l1*cos(q0), l1*sin(q0)];
    x_q2 = x_q1+l2.*[-cos(q1-q0), sin(q1-q0)];
    x_q3 = x_q2+l3.*[cos(q2-q1+q0), sin(q2-q1+q0)];
    x = [x_q0;x_q1;x_q2;x_q3]-[ones(4,1).*A(2), zeros(4,1)];
    
    x(:,2)=  x(:,2)+0.025;
    if nargin>2
        varargout{1} = plot(x(end,1),x(end,2),'*','color',varargin{1}); hold all;
    else
        varargout{1} = plot(x(end,1),x(end,2),'*'); hold all;
    end
    
    out1 = x(end,1);
    out2 = x(end,2);
    
    
end

