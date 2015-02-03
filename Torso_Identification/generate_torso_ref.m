function [pos,vel,acc,time] = generate_torso_ref(max_vel,max_acc,x_ref,dt_ref,varargin)

    lock = false;
    if nargin>4
        locked = varargin{1};
        lock = true;
    end
    
    n_ref = length(dt_ref);
    Ts = 0.001;
    
    x = [x_ref(1,:)];
    xd = [0,0];
    xdd = [0,0];
    t = [0];
    
    for i=1:1:n_ref
        n1 = 0;
        n2 = 0;
        
        % generate reference between 2 points
        if x_ref(i,1)~=x_ref(i+1,1)
            [x1_step,xd1_step,xdd1_step,t1_step] = reference_generator(x_ref(i,1),x_ref(i+1,1),Ts,max_vel,max_acc,max_acc/Ts);
            n1 = length(x1_step);
        end
        if x_ref(i,2)~=x_ref(i+1,2)
            [x2_step,xd2_step,xdd2_step,t2_step] = reference_generator(x_ref(i,2),x_ref(i+1,2),Ts,max_vel,max_acc,max_acc/Ts);
            n2 = length(x2_step);
        end
        
        if (lock)
            if locked(i)
                x2_step = shoulders_zero(x1_step);
                xd2_step = [diff(x2_step)./Ts;0];
                xdd2_step = [diff(xd2_step)./Ts;0];
                n2 = length(x2_step);
            end
        end
        
        if n1==0 && n2==0
            x_step = x_ref(i+1,:);
            xd_step = [0,0];
            xdd_step = xd_step;
            t_step = [0];
        else
            if n1>=n2
                x_step = [ones(n1,1).*x_ref(i+1,1),ones(n1,1).*x_ref(i+1,2)];
                xd_step = zeros(n1,2);
                xdd_step = xd_step;
                t_step = t1_step;
            else
                x_step = [ones(n2,1).*x_ref(i+1,1),ones(n2,1).*x_ref(i+1,2)];
                xd_step = zeros(n2,2);
                xdd_step = xd_step;
                t_step = t2_step;
            end
            
            if n1~=0
                x_step(1:n1,1) = x1_step;
                xd_step(1:n1,1) = xd1_step;
                xdd_step(1:n1,1) = xdd1_step;
            end
            if n2~=0
                x_step(1:n2,2) = x2_step;
                xd_step(1:n2,2) = xd2_step;
                xdd_step(1:n2,2) = xdd2_step;
            end
        end
        
        % fill array to the desired duration time
        if dt_ref(i)~=0
            while (t_step(end)<=dt_ref(i))
                x_step = [x_step;x_ref(i+1,:)];
                xd_step = [xd_step;0,0];
                xdd_step = [xdd_step;0,0];
                t_step = [t_step;t_step(end)+Ts];
            end
        end
        
        x = [x;x_step(2:end,:)];
        xd = [xd;xd_step(2:end,:)];
        xdd = [xdd;xdd_step(2:end,:)];
        t = [t;t(end)+t_step(2:end)];
    end

    pos = x;
    vel = xd;
    acc = xdd;
    time = t;
end
    