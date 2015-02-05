clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fit gravity compensation %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%


load data/sine_leg_up_up
joint = 1;
ref_0 = 0.38;
t_stop = 55;

% plot data
figure;
subplot(2,1,1); plot(time,ref);
subplot(2,1,2); plot(time,u);

% find start and end position
tstart = find(ref(:,joint)<ref_0-0.001,1,'first')
tend = find(ref(:,joint)>ref_0-0.001 & time<t_stop,1,'last')
subplot(2,1,1); hold all; 
plot(time(tstart),ref(tstart,joint),'r*',time(tend),ref(tend,joint),'r*');
% compute gravity term
u_g = mean(u(tstart:tend,joint))
subplot(2,1,2); hold all;
plot(time,ones(size(time)).*u_g);


%% Equilibrium model
load motion_Volt_angle
xsp1 = 0.38; xsp2 = 0.44;
qn = [spindle1_to_angle0(xsp1);spindle2_to_angle2(xsp2)];
qdn = [0;0];
S_n = double(subs(S,[q;qd],[qn;qdn]));
H_n = double(subs(H,[q;qd],[qn;qdn]));
tau_eq = inv(S_n)*H_n

plot(time,ones(size(time)).*tau_eq(joint),'r--');

all_grids_on();




