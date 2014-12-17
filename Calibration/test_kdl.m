clear all; close all; clc;

imu_q0 = 1.132;
% calculate angle 0
init_spindle1 = 0.24;
q0_guess = spindle1_to_angle0(init_spindle1);

e0 = imu_q0-q0_guess;

q0_new = q0_guess+e0
init_spindle1_new = angle0_to_spindle1(q0_new)
