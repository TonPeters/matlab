%%%% Calibration of the torso joints %%%%
clear all; close all; clc;

%% get orientation of the base plate
s = sprintf(['Determine the angle of the base plate:\n',...
    '\t place the IMU on the base plate of SERGIO,\n',...
    '\t or on the base plate of the external mount.\n']);
disp(s)

prompt = 'Enter the measured IMU angle? ';
base_angle = input(prompt);

%% get orientation of the laser
s = sprintf(['\n Determine the angle of the laser:\n',...
    '\t place the IMU on top of the laser,\n']);
disp(s)

prompt = 'Enter the measured IMU angle? ';
laser_angle = input(prompt);

%% get the calipher length
s = sprintf(['\n Determine the angles of the torso by listning to the topic,\n',...
    '\t rostopic echo /sergio/torso/measurements,\n']);
disp(s)

prompt = 'Enter the angle of the ankle joint (first one): ';
q0 = input(prompt);

prompt = 'Enter the angle of the knee joint (second one): ';
q1 = input(prompt);

prompt = 'Enter the angle of the ankle joint (third one): ';
q2 = input(prompt);


%% calculate the laser offset
laser_angle_wrt_floor = laser_angle-base_angle;
trunk_angle_wrt_floor = q0-q1+q2;
laser_angle_wrt_trunk = trunk_angle_wrt_floor-laser_angle_wrt_floor;

laser_offset_angle = pi/2-laser_angle_wrt_trunk

% pitch rotation of the laser tilt joint
pitch = pi/2-laser_offset_angle

%% result calibration laser joint
s = sprintf(['\n Calibrate the laser joint:\n',...
    '\t - go to the sergio_description directory,\n',...
    '\t - cd urdc/xacro/torso:\n',...
    '\t \t CALIPHER_OFFSET1 = %5.5f,\n',...
    '\t \t CALIPHER_OFFSET2 = %5.5f.\n',...
    'Now restart the hardware and calibration is done.\n',...
    '- Note, you can repeat these steps in another orientation to verify\n',...
    '  the results.\n',...
    ' Angle 0 = %4.4f, and Angle 2 = %4.4f degrees\n'],...
    offset_calipher1,offset_calipher2,q0/2/pi*360,q2/2/pi*360);
disp(s)





