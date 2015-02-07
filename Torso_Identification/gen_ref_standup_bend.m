%% Generate reference trajectory for the torso
clear all; 
% close all; 
clc;

%% load model
run torso_measures_NX

th_0_min = spindle1_to_angle0(min_spindle1); th_0_min = 0.15;
th_0_max = spindle1_to_angle0(max_spindle1);
th_2_min = spindle2_to_angle2(min_spindle2);
th_2_max = spindle2_to_angle2(max_spindle2);
Ts = 0.001;

th_0_center = (th_0_max+th_0_min)/2;
th_2_center = (th_2_max+th_2_min)/2;

q2_bottom = shoulders_zero(th_0_min);
q2_top = shoulders_zero(th_0_max);
q2_center = shoulders_zero(th_0_center);


% ref = [th_0_center, q2_center;...
%     th_0_center, q2_center;...
%     th_0_center, th_2_min;...
%     th_0_center, th_2_min;...
%     th_0_center, q2_center;...
%     th_0_center, q2_center];
% 
% dt_ref = [4;0;0.5;0;0.5]; %;0;0.5;0;0.5;0;0.5;0;0.5];
% lock   = [0;  0;0  ;0;0;]; %  1;0;  0;0;  0;0;  1;0];
% vel = 0.1;
% acc = 0.1;

ref = [th_0_min, q2_bottom;...
    th_0_min, q2_bottom;...
    th_0_min, th_2_min;...
    th_0_min, th_2_min;...
    th_0_min, q2_bottom;...
    th_0_min, q2_bottom;...
    th_0_max, q2_top;...
    th_0_max, q2_top;...
    th_0_max, th_2_min;...
    th_0_max, th_2_min;...
    th_0_max, q2_top;...
    th_0_max, q2_top;...
    th_0_min, q2_bottom;...
    th_0_min, q2_bottom];

dt_ref = [5.0;0;0.5;0;0.5;0;0.5;0;0.5;0;0.5;0;0.5];
lock   = [0;  0;0  ;0;0;  1;0;  0;0;  0;0;  1;0];
vel = 0.15;
acc = 0.15;

[q,qd,qdd,t] = generate_torso_ref(vel,acc,ref,dt_ref,lock);

figure
subplot(3,1,1); plot(t,q);
subplot(3,1,2); plot(t,qd);
subplot(3,1,3); plot(t,qdd);
linkaxes(get(gcf,'children'),'x')

save('reference_standup','q','qd','qdd','t');

%% generate spindle reference
sp1 = angle0_to_spindle1(q(:,1));
sp2 = angle2_to_spindle2(q(:,2));

figure
subplot(3,1,1); plot(t,sp1,t,sp2); title('spindle reference');ylabel('x spindle [m]');
subplot(3,1,2); plot_tdiff(t,sp1); hold all; plot_tdiff(t,sp2); ylabel('dx spindle [m/s]');
subplot(3,1,3); plot_tdiff(t,[diff(sp1)./Ts;0]); hold all; plot_tdiff(t,[diff(sp2)./Ts;0]);ylabel('ddx spindle [m/s2]');
linkaxes(get(gcf,'children'),'x')

all_grids_on();

%% save for cpp
file_dir = '/home/amigo/ros/data/private/Ton_data/torso_identification/reference/';
file_name = 'ref_stand_bend.txt';
fileID = fopen([file_dir,file_name],'wt');
n = length(t)
q(1,:)
sp1(1);
sp2(1);

formatSpec = '%9.8f\t%9.8f\n';
for i=1:1:n
    fprintf(fileID,formatSpec,sp1(i),sp2(i));
end

fclose(fileID);

% fprintf(formatSpec,