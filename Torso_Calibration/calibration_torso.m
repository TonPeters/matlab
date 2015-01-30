%%%% Calibration of the torso joints %%%%
clear all; close all; clc;

%% get orientation of the base plate
s = sprintf(['Determine the angle of the base plate:\n',...
    '\t place the IMU on the base plate of SERGIO,\n',...
    '\t or on the base plate of the external mount.\n']);
disp(s)

prompt = 'Enter the measured IMU angle? ';
B_angle = input(prompt);

%% get orientation of rear leg plate
s = sprintf(['\n Determine the angle of rear leg plate:\n',...
    '\t place the IMU on the rear leg plate,\n']);
disp(s)

prompt = 'Enter the measured IMU angle? ';
RL_angle = input(prompt);

%% get the calipher length
s = sprintf(['\n Determine the calipher length:\n',...
    '\t by topic?,\n']);
disp(s)

prompt = 'Enter the measured calipher length? ';
l_calipher1 = input(prompt);

%% calculate the spindle and spring length
% angle of ankle joint
q0 = imu_angle0_to_angle0(abs(RL_angle-B_angle));

% spindle length
l_spindle1 = angle0_to_spindle1(q0);
% spring length (calipher is mounted to this joint)
l_spring1 = angle0_to_spring1(q0);

%% calculate the offset of the calipher
offset_calipher1 = l_spring1-l_calipher1;
s = sprintf(['\n The offset of calipher 1 should be:\n',...
    '\t %5.4f,\n'],offset_calipher1);
disp(s)

%% get orientation of rear leg plate
s = sprintf(['\n Determine the angle of torso back plate:\n',...
    '\t place the IMU on the torso back plate,\n']);
disp(s)

prompt = 'Enter the measured IMU angle? ';
T_angle = input(prompt);

%% get the calipher length
s = sprintf(['\n Determine the calipher length:\n',...
    '\t by topic?,\n']);
disp(s)

prompt = 'Enter the measured calipher length? ';
l_calipher2 = input(prompt);

%% calculate the spindle and spring length
% angle of hip joint
q2 = imu_angle2_to_angle2(abs(T_angle-B_angle),q0);

% spindle length
l_spindle2 = angle2_to_spindle2(q2);
% spring length (calipher is mounted to this joint)
l_spring2 = angle2_to_spring2(q2);

%% calculate the offset of the calipher
offset_calipher2 = l_spring2-l_calipher2;
s = sprintf(['\n The offset of calipher 2 should be:\n',...
    '\t %5.4f,\n'],offset_calipher2);
disp(s)


%% Results
s = sprintf(['\n Calibrate the caliphers:\n',...
    '\t - go to the sergio_hardware directory,\n',...
    '\t - in the torso_controller.ops file adjust:\n',...
    '\t \t CALIPHER_OFFSET1 = %5.5f,\n',...
    '\t \t CALIPHER_OFFSET2 = %5.5f.\n',...
    'Now restart the hardware and calibration is done.\n',...
    '- Note, you can repeat these steps in another orientation to verify\n',...
    '  the results.\n',...
    ' Angle 0 = %4.4f, and Angle 2 = %4.4f degrees\n'],...
    offset_calipher1,offset_calipher2,q0/2/pi*360,q2/2/pi*360);
disp(s)





