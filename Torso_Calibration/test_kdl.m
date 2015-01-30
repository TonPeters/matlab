clear all; close all; clc;

imu_q0 = 1.132;
% calculate angle 0
init_spindle1 = 0.24;
q0_guess = spindle1_to_angle0(init_spindle1);

e0 = imu_q0-q0_guess;

q0_new = q0_guess+e0
init_spindle1_new = angle0_to_spindle1(q0_new)


%%
q = [1.0; 1.0]
sp = [angle0_to_spindle1(q(1));angle2_to_spindle2(q(2))]
angl = [spindle1_to_angle0(sp(1));spindle2_to_angle2(sp(2))]
an2 = [angl(1);angle0_to_angle1(angl(1));angl(2)]
