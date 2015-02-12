clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fit gravity compensation %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pos1 = {'up','c','do'};
ref1 = [0.38, 0.325, 0.27];
pos2 = {'up','c','do'};
ref2 = [0.44, 0.405, 0.37];
joint = 1;

tsta = [0, 5, 0;0, 0, 0;0, 0, 0];
tstop = [55, 60, 55;55, 55, 55;55, 60, 55];

format_string = 'Equilibrium Voltage legs, %2s, %2s, model = %5.4f, measurement = %5.4f';
tau_mod = zeros(3);
tau_meas = tau_mod;
for i=1:1:3
    for j=1:1:3
        if (i==3 && j<3)
            continue;
        end
        
        load(['data/sine_leg_',pos1{i},'_',pos2{j}]);
        

        ref_0 = ref1(i);
        t_stop = tstop(i,j);
        t_start = tsta(i,j);

        % plot data
        figure;
        subplot(2,1,1); plot(time,ref);
        subplot(2,1,2); plot(time,u);

        % find start and end position
        tstart = find(ref(:,joint)<ref_0-0.001 & time>t_start,1,'first');
        tend = find(ref(:,joint)>ref_0-0.001 & time<t_stop,1,'last');
        subplot(2,1,1); hold all; 
        plot(time(tstart),ref(tstart,joint),'r*',time(tend),ref(tend,joint),'r*');
        % compute gravity term
        tau_meas(i,j) = mean(u(tstart:tend,joint));
        subplot(2,1,2); hold all;
        plot(time,ones(size(time)).*tau_meas(i,j));


        %% Equilibrium model
        load /home/ton/Dropbox/Linux/Matlab/Torso/Model/motion_Volt_angle
        xsp1 = ref1(i); xsp2 = ref2(j);
        qn = [spindle1_to_angle0(xsp1);spindle2_to_angle2(xsp2)];
        qdn = [0;0];
        S_n = double(subs(S,[q;qd],[qn;qdn]));
        H_n = double(subs(H,[q;qd],[qn;qdn]));
        tau_eq = inv(S_n)*H_n;
        tau_mod(i,j) = tau_eq(joint);
        
        str = sprintf(format_string,pos1{i},pos2{j},tau_eq(joint),tau_meas(i,j));
        disp(str)
        
        plot(time,ones(size(time)).*tau_eq(joint),'r--');
    end
end
    %%
figure;
for i=1:1:3
    if i==3
        plot(ref1,tau_mod(:,i),'-*',ref1,tau_meas(:,i),'--*'); hold all;
    else
        plot(ref1(1:2),tau_mod(1:2,i),'-*',ref1(1:2),tau_meas(1:2,i),'--*'); hold all;
    end
end
xlabel('Spindle length [m]'); ylabel('Input Voltage');
legend('mod up','meas up','mod c','meas c','mod do','meas do')
    
all_grids_on();    
all_grids_on();




