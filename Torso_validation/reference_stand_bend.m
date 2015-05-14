clear all; close all; clc;

run torso_measures_NX;
min_spindle1 = 0.260;
max_spindle1 = 0.395;
min_spindle2 = 0.361;

th_0_min = spindle1_to_angle0(min_spindle1);
th_0_max = spindle1_to_angle0(max_spindle1);
th_2_min = spindle2_to_angle2(min_spindle2);
th_2_max = spindle2_to_angle2(max_spindle2);

q0 = [th_0_min,th_0_max,th_0_max,th_0_max,th_0_min,th_0_min,th_0_min].';
q2 = shoulders_zero(q0);
q2([3,6]) = [th_2_min,th_2_min];

x0 = angle0_to_spindle1(q0);
x2 = angle2_to_spindle2(q2);

n = length(q0);
format_str = 'q0 = %3.2f, q2 = %3.2f, l1 = %5.4f, l2 = %5.4f\n';
for i=1:n
    fprintf(format_str,q0(i),q2(i),x0(i), x2(i));
end