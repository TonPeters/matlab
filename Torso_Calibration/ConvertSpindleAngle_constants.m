%%%% Calculate constants for calibration component %%%%
clear all; close all; clc;
run torso_measures_NX

% vector containing constants
CC = zeros(14,1);

% vector with origin in A and pointing in negative y dircetion
AZ = A+[0;-1;0];
%% conversion spindle1 to angle0
% lengths
AE = di(A,E);
AC = di(A,C);
% angles
CAZ = an(C,A,AZ);

% constants
CC(1) = AE^2+AC^2;
CC(2) = -2*AE*AC;
CC(3) = CAZ;

%% conversion spindle2 to angle2
% lengths
HJ = di(H,J);
JK = di(J,K);
% angles
GJH = an(G,J,H);
KJL = an(K,J,L);

% constants
CC(4) = HJ^2+JK^2;
CC(5) = -2*HJ*JK;
CC(6) = GJH+KJL;

%% conversion angle0 to angle1
% lengths
AB = di(A,B);
AG = di(A,G);
BD = di(B,D);
GD = di(G,D);
% angles
BAZ = an(B,A,AZ);
JGD = an(J,G,D);


% constants
CC(7) = AB^2+AG^2;
CC(8) = 2*AB*AG;
CC(9) = BAZ;
CC(10) = AB^2-AG^2;
CC(11) = -2*AG;
CC(12) = BD^2-GD^2;
CC(13) = -2*GD;
CC(14) = JGD;

%% conversion spring1 to spindle1
% lengths
AF = di(A,F);

% constants
CC(15) = AB^2+AF^2;
CC(16) = -2*AB*AF;

%% conversion spring2 to spindle2
% lengths
IJ = di(I,J);

% angles
GJI = an(G,J,I);

% constants
CC(17) = IJ^2+JK^2;
CC(18) = -2*IJ*JK;
CC(19) = GJI+KJL;

%% angle limits in calculation spindle length from angles
CC(20) = pi-CAZ; % angle 0 max 
CC(21) = -CAZ; % angle 0 min
CC(22) = pi+GJH+KJL; % angle 2 max
CC(23) = GJH+KJL; % angle 2 min

%% print results to copy paste
print_format = 'C%.i = %8.6f;';
disp(' Constants used in ConversionSpindleAngle.*:');
const = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,20,21,22,23];
for i=const
    string = sprintf(print_format,i,CC(i));
    disp(string)
end


disp(sprintf('\n Constants used in InitCalipherToSpindle.*:'));
const = [1,2,3,4,5,6,9,15,16,17,18,19];
for i=const
    string = sprintf(print_format,i,CC(i));
    disp(string)
end
