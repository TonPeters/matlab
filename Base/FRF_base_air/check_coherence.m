clear all; 
% close all; 
clc;

%% hardware info
WHEELRAD 			= 0.075*1.0147; %	# Corrected for effective wheel radius
GEARRATIO 			= 9.0/169.0;	%	# Gearratio Maxxon GP42C
BITS2CURRENT 		= 50.0/2046.0; 	%	# Bit value to current value (+-2046 bit FPGA-> +-5V ELMO-> +-50A)
CURRENT2TORQUE 		= 29.2e-3; 		%	# Torque constant of the motor Maxxon RE35
BITS2WHEELTORQUE 	= BITS2CURRENT * CURRENT2TORQUE * 1.0/GEARRATIO;
TWOPI 			= 2.0*3.141592;%
ENCODERCOUNTS 	= 500.0*4.0;	%	# Counts per cycle
ENCODERBITS 	= 65536;		%	# From encoder datasheet
ENC2RAD 		= TWOPI*GEARRATIO/ENCODERCOUNTS;

%% motor specs
nom_torque = 0.101;
max_torque = 1.200;
nom_current = 3.62;
max_current = 41.4;
res_current = 1*BITS2CURRENT;
res_torque = res_current*CURRENT2TORQUE;



%% encoder specs
% resolution
min_vis_bit = 1;                                    % 1 bit
res_rad = min_vis_bit*ENC2RAD;                  % 0.0001673 rad

%% Plant visibility
res_transfer = db(res_rad/res_torque);   %  dB
nom_transfer = db(res_rad/nom_torque);  %  dB
max_transfer = db(res_rad/max_torque);  % dB


fprintf('%13s | resolution |  nominal | maximal  |\n','')
formatspec = '%13s | % 10.4f | % 8.3f | % 8.3f |\n';
fprintf(formatspec,'torque [Nm]',res_torque,nom_torque,max_torque);
fprintf(formatspec,'current [A]',res_current,nom_current,max_current);
fprintf(formatspec,'transfer [dB]',res_transfer,nom_transfer,max_transfer);

fprintf('\n Encoder resolution: %7.3e [rad] \n',res_rad);



