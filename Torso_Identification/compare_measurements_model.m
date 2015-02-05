clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compare measurements to model %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load manual_ref07;
load motion_Volt_angle;

%% compute reference trajectory
Ts = 0.001;

nq = length(q);
q_num1 = spindle1_to_angle0(ref(:,1));
q_num2 = spindle2_to_angle2(ref(:,2));
q_num = [q_num1,q_num2].';
qd_num = [zeros(nq,1),diff(q_num,1,2)./Ts];
qdd_num = [zeros(nq,1),diff(qd_num,1,2)./Ts];

indices = find(rem(time,0.1)==0);
time2 = time(indices);
q_num = q_num(:,indices);
qd_num = qd_num(:,indices);
qdd_num = qdd_num(:,indices);

n = length(time2);

% check reference trajectory
figure;
subplot(3,1,1);plot(time2,q_num.');
subplot(3,1,2);plot(time2,qd_num.');
subplot(3,1,3);plot(time2,qd_num.');

%% compute motor Voltage
tau_model = zeros(nq,n);
S_num = double(S);
S_inv = inv(S_num);

% for i=1:1:n
%     clc; i
%     M_num = double(subs(M,[q,qd],[q_num(:,i),qd_num(:,i)]));
%     H_num = double(subs(H,[q,qd],[q_num(:,i),qd_num(:,i)]));
%     
%     tau_model(:,i) = S_inv*(M_num*qdd_num(:,i)+H_num);
% end
% save('volt_model','tau_model');
load volt_model


%% compute measurement torque
tau_meas = u;

%% plot results
figure;
subplot(2,1,1); plot(time2,q_num);
ylabel('Ref angle [rad]');
subplot(2,1,2); plot(time,tau_meas,time2,tau_model);
ylabel('Motor torque [Nm]'); xlabel('Time [s]');
linkaxes(get(gcf,'children'),'x');

all_grids_on();

