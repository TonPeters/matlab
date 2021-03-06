clear all; 
close all; 
clc;

load 'add_mass/upper_10_mass01';
ref10 = ref(:,2);
time10 = time;
enc10 = enc(:,2);
u10 = u(:,2);
err10 =ref10-enc10;
i10 = find(ref10>0.41,1,'first');
t10 = time10(i10);
th0 = spindle1_to_angle0(mean(ref(:,1)));
th1 = angle0_to_angle1(th0);
angle_offset = th0-th1;
load 'add_mass/upper_0_mass01';
ref0 = ref(:,2);
time0 = time;
enc0 = enc(:,2);
u0 = u(:,2);
err0 = ref0-enc0;
i0 = find(ref0>0.41,1,'first');
t0 = time0(i0);
load 'add_mass/upper_20_mass01';
ref20 = ref(:,2);
time20 = time;
enc20 = enc(:,2);
u20 = u(:,2);
err20 = ref20-enc20;
i20 = find(ref20>0.41,1,'first');
t20 = time20(i20);

load 'add_mass/m10kg_vel004_leg02';
ref102 = ref(:,2);
time102 = time;
enc102 = enc(:,2);
u102 = u(:,2);
err102 =ref102-enc102;
i102 = find(ref102>0.41,1,'first');
t102 = time10(i102);
th02 = spindle1_to_angle0(mean(ref(:,1)));
th12 = angle0_to_angle1(th02);
angle_offset2 = th02-th12;
load 'add_mass/m0kg_vel004_leg02';
ref02 = ref(:,2);
time02 = time;
enc02 = enc(:,2);
u02 = u(:,2);
err02 = ref02-enc02;
i02 = find(ref02>0.41,1,'first');
t02 = time02(i02);

%% plot results
n_plots = 3; i_p = 1;
figure; 
subplot(n_plots,1,i_p); i_p = i_p+1;
plot(time0-t0,ref0,time10-t10,ref10,time20(1:64000)-t20,ref20(1:64000)); ylabel('ref [m]'); grid on; hold all;
plot(time0-t0,enc0,time10-t10,enc10,time20(1:64000)-t20,ref20(1:64000)); legend('ref1','ref2','enc1','enc2');
subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time0-t0,u0,time10-t10,u10,time20(1:64000)-t20,u20(1:64000)); ylabel('control [V]'); grid on; 
% subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,enc1,time,enc2); ylabel('enc [m]'); grid on;
subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time0-t0,err0,time10-t10,err10,time20(1:64000)-t20,err20(1:64000)); ylabel('err [m]'); grid on;
linkaxes(get(gcf,'children'),'x');

%%
figure;
plot((spindle2_to_angle2(enc0)+angle_offset)./pi*180,u0,(spindle2_to_angle2(enc10)+angle_offset)./pi*180,u10,...
    (spindle2_to_angle2(enc20(1:64000))+angle_offset)./pi*180,u20(1:64000)); grid on; hold all;
plot((spindle2_to_angle2(enc02)+angle_offset2)./pi*180,u02,(spindle2_to_angle2(enc102)+angle_offset2)./pi*180,u102);
xlabel('angle w.r.t. floor [deg]'); ylabel('input1 V');


%% save
% th_0k = (spindle2_to_angle2(enc0)+angle_offset)./pi*180;
% th_10k= (spindle2_to_angle2(enc10)+angle_offset)./pi*180;
% th_20k= (spindle2_to_angle2(enc20(1:64000))+angle_offset)./pi*180;
% u_0k = u0;
% u_10k = u10;
% u_20k = u20(1:64000);
% 
% save('add_mass/upper_3mass.mat','th_0k','th_10k','th_20k','u_0k','u_10k','u_20k');

% th_0k2 = (spindle2_to_angle2(enc02)+angle_offset2)./pi*180;
% th_10k2= (spindle2_to_angle2(enc102)+angle_offset2)./pi*180;
% u_0k2 = u02;
% u_10k2 = u102;
% 
% save('add_mass/upper_2mass_leg02.mat','th_0k2','th_10k2','u_0k2','u_10k2');

% % add dependency on angle force applied
% th2_0 = spindle2_to_angle2(enc0);
% red0 =  -0.4715.*th2_0.*th2_0+  1.516.*th2_0-0.2201;
% th2_10 = spindle2_to_angle2(enc10);
% red10 =  -0.4715.*th2_10.*th2_10+  1.516.*th2_10-0.2201;
% hold all;
% plot((spindle2_to_angle2(enc0)+angle_offset)./pi*180,u0.*red0,(spindle2_to_angle2(enc10)+angle_offset)./pi*180,u10.*red10); grid on;
% xlabel('angle w.r.t. floor [deg]'); ylabel('input1 V');




all_grids_on();