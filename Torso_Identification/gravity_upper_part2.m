clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fit gravity compensation %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%


load data/sine_tr_up_up
joint = 2;
ref_0 = 0.439;
t_stop = 115;

% plot data
figure;
subplot(2,1,1); plot(time,ref);
subplot(2,1,2); plot(time,u);

% find start and end position
tstart = find(ref(:,joint)<ref_0-0.001,1,'first');
tend = find(ref(:,joint)>ref_0-0.001 & time<t_stop,1,'last');
subplot(2,1,1); hold all; 
plot(time(tstart),ref(tstart,joint),'r*',time(tend),ref(tend,joint),'r*');
% compute gravity term
u_g = mean(u(tstart:tend,joint))
subplot(2,1,2); hold all;
plot(time,ones(size(time)).*u_g,'k');

%% Equilibrium model
load motion_Volt_angle
xsp1 = 0.38; xsp2 = 0.44;
qn = [spindle1_to_angle0(xsp1);spindle2_to_angle2(xsp2)];
qdn = [0;0];
S_n = double(subs(S,[q;qd],[qn;qdn]));
H_n = double(subs(H,[q;qd],[qn;qdn]));
tau_eq = inv(S_n)*H_n;
tau_eq(joint)

plot(time,ones(size(time)).*tau_eq(joint),'r--');

%% simulate sine
% load motion_Volt_angle
% t = linspace(0,10,20).';
% Fs = 0.1;
% sine = 0.44+0.01*sin(t*Fs*2*pi);
% q_num = spindle2_to_angle2(sine);
% qd_num = [diff(q_num);0];
% qdd_num = [diff(qd_num);0];
% q1 = spindle1_to_angle0(0.38);
% 
% tau_model = zeros(size(t));
% S_inv = inv(S);
% for i=1:1:length(t)
%     clc; i
%     M_num = double(subs(M,[q;qd],[q1;q_num(i);0;qd_num(i)]));
%     H_num = double(subs(H,[q;qd],[q1;q_num(i);0;qd_num(i)]));
%     
%     tau_tmp = S_inv*(M_num*[0;q_num(i)]+H_num);
%     tau_model(i) = tau_tmp(2);
% end
% 
% subplot(2,1,2); plot(t+10,tau_model);

all_grids_on();




