function out = plot_torso_pos2(q0,q2,varargin)


    plotsettings
    run torso_measures_NX
    l1 = di(A,G);
    l2 = di(G,J);       % length from knee to hip
    lcm2 = l2-di(D,J)/2;    % length rotation point to center of mass (half of leg length)
    l3 = di(J,L);       % length from hip to shoulder
    lcm3 = di(J,L)/2;
    lFsp1 = di(A,E);
    lFsp2 = di(J,K);
    
    q1 = angle0_to_angle1(q0);
    
    % positions of the joints
    x_q0 = [0 0];
    x_q1 = [l1*cos(q0), l1*sin(q0)];
    x_q2 = x_q1+l2.*[-cos(q1-q0), sin(q1-q0)];
    x_q3 = x_q2+l3.*[cos(q2-q1+q0), sin(q2-q1+q0)];
    x = [x_q0;x_q1;x_q2;x_q3]-[ones(4,1).*A(2), zeros(4,1)];
    
    
    if nargin>2
        out = plot(x(:,1),x(:,2),'-o','color',varargin{1}); hold all;
    else
        out = plot(x(:,1),x(:,2),'-o'); hold all;
    end
    
    if nargin>3
        if varargin{2}
            % resize axes
            ax = gca;
            yl= get(ax,'ylim');
            xl = get(ax,'xlim');
            set(ax,'units','centimeters');
            pos = get(ax,'position');
            pos(3) = pos(1)+diff(xl)*10;
            pos(4) = pos(2)+diff(yl)*10;
            set(ax,'position',pos)

            % resize figure
            h = gcf;
            set(h,'units','centimeters');
            fpos = get(h,'position');
            fpos(3) = pos(1)+pos(3)+1;
            fpos(4) = pos(2)+pos(4)+1;
            set(h,'position',fpos);
        end
    end
    
end

