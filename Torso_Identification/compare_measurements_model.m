clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compare measurements to model %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load manual_ref07;
load motion_motor_model;
load motor_constants;

%% compute reference trajectory
Ts = 0.001;

nq = length(q);
q_num1 = spindle1_to_angle0(ref(:,1));
q_num2 = spindle2_to_angle2(ref(:,2));
q_num = [q_num1,q_num2].';
qd_num = [zeros(nq,1),diff(q_num,1,2)./Ts];
qdd_num = [zeros(nq,1),diff(qd_num,1,2)./Ts];
thm1 = q_num(1,:).*r1;
thm2 = q_num(2,:).*r2;
thm = [thm1;thm2];
thdm = [zeros(2,1),diff(thm,1,2)./Ts];
thddm = [zeros(2,1),diff(thdm,1,2)./Ts];

indices = find(rem(time,0.1)==0);
time2 = time(indices);
q_num = q_num(:,indices);
qd_num = qd_num(:,indices);
qdd_num = qdd_num(:,indices);
thm = thm(:,indices);
thdm = thdm(:,indices);
thddm = thddm(:,indices);

n = length(time2);

% check reference trajectory
figure;
subplot(3,1,1);plot(time2,q_num.');
subplot(3,1,2);plot(time2,qd_num.');
subplot(3,1,3);plot(time2,qd_num.');

%% compute model motor torque
% tau_model = zeros(nq,n);
% for i=1:1:n
%     clc; i
%     tau_model(:,i) = double(subs(Mc*q+Hc,[q,qd,qdd],[q_num(:,i),qd_num(:,i),qdd_num(:,i)]));
% end
% save('tau_model','tau_model');
load tau_model

%% compute motor torque
tau_motor = zeros(2,n);
for i=1:1:n
    tau_motor(:,i) = Jm*thddm(:,i)+Bm*thdm(:,i);
end
tau_motor = tau_motor./Kelm;

%% compute measurement torque
tau_meas = u;

%% plot results
figure;
subplot(2,1,1); plot(time2,q_num);
ylabel('Ref angle [rad]');
subplot(2,1,2); plot(time,tau_meas,time2,tau_motor);
ylabel('Motor torque [Nm]'); xlabel('Time [s]');
linkaxes(get(gcf,'children'),'x');

all_grids_on();

