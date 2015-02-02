function [q,qd,qdd,time] = reference_generator(xstart,xend,Ts,vel,acc,jerk)
% REFERENCE_GENERATOR, creates a reference trajectory using a constant jerk  
%     and a maximum velocity and acceleration. 
%   
%  [pos,vel,acc,time] = REFERENCE_GENERATOR(xstart,xend,Ts,vel_max,acc_max,jerk)

    
    tj = acc/jerk;          % constant jerk time (zero -> max acc)
    ta = vel/acc-acc/jerk;  % constant acc. time (jerk vel -> max vel -jerk vel)
    
    jerk_used = jerk;
    % set direction
    if (xstart>xend), jerk_used = -jerk_used; end
    
    a = zeros(4,7);                             % integral constants for each part of the trajectory
    t_steps = [0;tj;tj+ta;2*tj+ta;0;0;0;0];     % time steps of the trajectory
    jerk_steps = [jerk_used;0;-jerk_used;0;-jerk_used;0;jerk_used]; % jerks for the trajectory
    xd_steps = zeros(3,1);                      % distance traveled in the first 3 parts of the trajectory
    q_1 = xstart; q_2 = 0; q_3 = 0;             % initial conditions
    
    for i=1:1:3
        a(:,i) = compute_integral_constants(t_steps(i),q_1,q_2,q_3,jerk_steps(i));  % generate integral constants
        [q_1,q_2,q_3] = ref_evaluate(a(:,i),t_steps(i+1));                          % evaluate integral on switch time
        xd_steps(i) = q_1;                                                          % store traveled distance at switch time  
    end
    
    xj_startup = xd_steps(1)-xstart+xd_steps(3)-xd_steps(2);   % distance traveled during constant jerk
    xa_startup = xd_steps(2)-xd_steps(1);               % distance traveled during constant acceleration
    xd_startup = 2*(xj_startup+xa_startup);             % distance traveled by accelerating and decelerating
    xd_startup = abs(xd_startup);
    
    xd = abs(xstart-xend); % distance to be traveled
    if xd_startup < xd % maximum acceleration and maximum velocity are reachable
        
        tv = (xd-xd_startup)/vel;   % constant velocity time
        t_total = 4*tj+2*ta+tv;     % total time
        
        % create output arrays
        time = (0:Ts:t_total).';
        q = zeros(size(time)); 
        qd = q;
        qdd = q;
        q(1) = xstart; % initial condition

        % complete swithc times and integral settings
        t_steps(5) = t_steps(4)+tv;
        t_steps(6) = t_steps(5)+tj;
        t_steps(7) = t_steps(6)+ta;
        t_steps(8) = t_steps(7)+tj;
        for i=4:1:7
            a(:,i) = compute_integral_constants(t_steps(i),q_1,q_2,q_3,jerk_steps(i));
            [q_1,q_2,q_3] = ref_evaluate(a(:,i),t_steps(i+1)); 
        end
        
        % generate the output data
        for i=1:1:7
            indices = find(time>t_steps(i) & time<=t_steps(i+1));
            [q(indices),qd(indices),qdd(indices)] = ref_evaluate(a(:,i),time(indices));
        end
        
    else
        % reduce velocity or acceleration
        
        if (abs(2*xj_startup) > xd) 
            % acceleration to high
            disp('reduce acceleration');
            acc_reduced = 0.9*acc;
            [q,qd,qdd,time] = reference_generator(xstart,xend,Ts,vel,acc_reduced,jerk);
        else
            % velocity to high
            disp('reduce velocity');
            vel_reduced = 0.9*vel;
            [q,qd,qdd,time] = reference_generator(xstart,xend,Ts,vel_reduced,acc,jerk);
        end
    
    end
end


function [pos,vel,acc] = ref_evaluate(a,time)
    pos = a(1)+a(2).*time+1/2*a(3).*time.^2+1/6*a(4).*time.^3;
    vel = a(2)+a(3).*time+1/2*a(4).*time.^2;
    acc = a(3)+a(4).*time;
end

function a = compute_integral_constants(t,q,qd,qdd,jerk)
    a = zeros(4,1);
    a(4) = jerk;
    a(3) = qdd-a(4)*t;
    a(2) = qd-a(3)*t-1/2*a(4)*t^2;
    a(1) = q-a(2)*t-1/2*a(3)*t^2-1/6*a(4)*t^3;
end
    
    
    
    
    
    