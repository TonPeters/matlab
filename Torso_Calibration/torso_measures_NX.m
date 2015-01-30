% clear all; close all; clc;
% get the measures from the design
% x is to the right, y is to the back, z is to te top

%% data from 3D model
% !! Use base top plate as reference for height
z_base_plate = 84.3;

% compute center shift points (set origin x,y to the wheel base center)
xcenter = (-29.7+220.3)/2;
xcenter2 = 94.8.*1e-3;
ycenter = (144.8-105.2)/2;

% base laser x y location
B_laser = [0; (-283.2-336.2)/2-ycenter].*1e-3; % z to be determined

% Wheel positions (from design)
W_1 = [275 -265].'.*1e-3;           % z to be determined
W_2 = [-275 -265].'.*1e-3; 
W_3 = [-275 265].'.*1e-3;
W_4 = [275 265].'.*1e-3;

% distance to front back left and right of the bottem plate
BP_left = [0; 406.8-xcenter; 0].*1e-3;
BP_right = [0; -216.2-xcenter; 0].*1e-3;
BP_front = [0; -313.2-ycenter; 0].*1e-3;
BP_back = [0; 399.8-ycenter; 0].*1e-3;
BP_bottom = [0;0;-11].*1e-3;


% torso points
A=[0		204.8-ycenter	140.8-z_base_plate].'.*1e-3;
B=[0		104.7-ycenter	46.3-z_base_plate].'.*1e-3;
C=[0		70.8-ycenter	46.3-z_base_plate].'.*1e-3;
D=[0		-74.3-ycenter	373.0-z_base_plate].'.*1e-3;
E=[0		-16.6-ycenter	413.1-z_base_plate].'.*1e-3;
F=[0		-29.8-ycenter	429.4-z_base_plate].'.*1e-3;
G=[0		-41.2-ycenter	443.4-z_base_plate].'.*1e-3;
H=[0		-74.8-ycenter	474.4-z_base_plate].'.*1e-3;
I=[0		-61.4-ycenter	496.1-z_base_plate].'.*1e-3;
J=[0		213.5-ycenter	766.6-z_base_plate].'.*1e-3;
K=[0		185.6-ycenter	840.5-z_base_plate].'.*1e-3;
L=[0		89.8-ycenter	1220.0-z_base_plate].'.*1e-3; % center of the soulder rotation
L2 = [0		95.0-ycenter	1200.8-z_base_plate].'.*1e-3; % center of the shoulder hole

% laser tilt points
Q=[0		60.6-ycenter	1013.4-z_base_plate].'.*1e-3;
R=[0		145.8-ycenter	1048.0-z_base_plate].'.*1e-3;
S=[0		153.8-ycenter	1114.5-z_base_plate].'.*1e-3;
T=[0		119.4-ycenter	1117.1-z_base_plate].'.*1e-3;
U=[0		73.5-ycenter	1120.6-z_base_plate].'.*1e-3;
V=[0		111.4-ycenter	1050.6-z_base_plate].'.*1e-3;
W=[0		106.5-ycenter	1009.9-z_base_plate].'.*1e-3;

% torso back plate
TB_bottom   = [0 239.6-ycenter	767.9-z_base_plate].'.*1e-3;
TB_top      = [0 177.5-ycenter	1165.1-z_base_plate].'.*1e-3;

% rear lag back plate
RL_bottom   = [0 191.9-ycenter   181.2-z_base_plate].'.*1e-3;
RL_top      = [0 -13.4-ycenter   433.8-z_base_plate].'.*1e-3;

% Head
HT_bottom   =[94.8-xcenter2	67.1-ycenter	1282.0-z_base_plate].'.*1e-3;
HT_top      =[94.8-xcenter2	52.1-ycenter	1325.6-z_base_plate].'.*1e-3;
% HP_right    =[73.7-xcenter2	55.4-ycenter	1352.1-z_base_plate].';
% HP_left     =[114.0-xcenter2	34.1-ycenter	1344.8-z_base_plate].';
HP_right = [74.3-xcenter2; 55.1-ycenter; 1352.0-z_base_plate].*1e-3;
HP_left = [115.3-xcenter2; 33.4-ycenter; 1344.6-z_base_plate].*1e-3;
HPT_center = (HP_left+HP_right)./2;

% Kinect			
HK_mount   =[93.6-xcenter2	30.7-ycenter	1381.6-z_base_plate].'.*1e-3;

%%%%%%%%%%%%%%%%%%% below follow calculations %%%%%%%%%%%%%%%%%%%%%%%%%%

%% calculate origin head center w.r.t. hip coordinates
angle_HC = an(L,J,HPT_center);
HPT_x = cos(angle_HC)*di(J,HPT_center);
HPT_z = -sin(angle_HC)*di(J,HPT_center);

%% calculate head pan rotation axes w.r.t hip coordinates
rotation_pan = an(HPT_center-L,[0;0;0],L-J);

%% distance from head center rotation point to kinect mount
K_z = di(HPT_center,HK_mount);

%% calculate torso laser rotation center w.r.t. hip coordinates
% center
TL_axes = V+(U-T);
% angle w.r.t. hip
angle_TL_axes = an(L,J,TL_axes);
TL_x = cos(angle_TL_axes)*di(J,TL_axes);
TL_z = -sin(angle_TL_axes)*di(J,TL_axes);

% translation from center of rotation to center laser
dist_center_to_rotation = di(U,Q)/2-di(T,V);
dist_center_laser_to_source = 11.5;

offset_laser = dist_center_to_rotation+dist_center_laser_to_source;


%% torso masses
M_LFL  = 1;
M_LRL  = 2.5;
M_UL   = 5;
M_T    = 30+10;

%% Gas spring constants
% legs
B_leg  = 404e-3+2*30e-3;
A_leg  = 175e-3;
F1_leg = 500;
X_leg  = 1.35;
F2_leg = F1_leg*X_leg;
K_leg  = (F2_leg-F1_leg)/(A_leg-10e-3);
FR_leg = 60;
L0_leg = B_leg  + (F1_leg+FR_leg/2)/K_leg;
Nsprings_leg = 2;

% trunk
B_trunk  = 404e-3+2*30e-3;  % complete extended length m
A_trunk  = 175e-3;          % spring motion range m
F1_trunk = 800;             % force min length
X_trunk  = 1.35;            % f2/f1
F2_trunk = F1_trunk*X_trunk;% force max length
K_trunk  = (F2_trunk-F1_trunk)/(A_trunk-10e-3); % spring constant (slope N/m)
FR_trunk = 60;              % direction dependant force
L0_trunk = B_trunk  + (F1_trunk+FR_trunk/2)/K_trunk; % length at zero energy m (not reachable)
N_springs_trunk = 0;        % amount of springs used

%% limits
max_spindle1 = 0.4047; % max is 0.41 is angle0 1.11 and angle1 2.5884
min_spindle1 = 0.2387; %0.2366 is angle0 0.0
init_spindle1 = di(C,E);
max_spindle2 = 0.460; % 0.489 is the absolute maximum (equilibrium point)
min_spindle2 = 0.36; % 0.3315 is the absolut minimum (equilibrium point)
% angle2 min = 0.2175, max = 3.2276 (mathematical limitation)
init_spindle2 = di(I,K);
