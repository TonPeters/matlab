clear all; 
% close all; 
clc;
plotsettings
%% on sergio
% filedir = '/home/amigo/ros/data/private/Ton_data/torso_validation/';
%% on my pc
filedir = '/home/ton/ros/data/private/Ton_data/torso_validation/';


filename = 'Trunk_new_acc15_leg10';
joint =2;
% filename = 'Leg_new_acc15_trunk10';
% joint =1;

data = importdata([filedir,filename,'.dat']);
vectorsizes = [2,2,2,2,2];
t_start = 0;
t_end = length(data.data(:,1));

GEARRATIO 			= 9.0/169.0;		
BITS2CURRENT 		= 25.0/2046.0; 		
CURRENT2TORQUE 		= 29.2e-3; 			
BITS2WHEELTORQUE 	= BITS2CURRENT * CURRENT2TORQUE * 1.0/GEARRATIO;
ENC2RAD = 2.0*3.141592*9.0/169.0/(500.0*4.0);
Kelm = 10;
facc = 0.8*0.024;
K_m = 29.2e-3;              % Nm/A,     torque constant
K_elm = 10;                  % Gain bij Elmo violin
    
sample_i = data.data(1:t_end,1);
trace_count = 1;
signal_count = 1;
for i=2:1:sum(vectorsizes)+1
    if signal_count > vectorsizes(trace_count)
        trace_count = trace_count+1;
        signal_count = 1;
    end
    trace{trace_count}.signal{signal_count} = data.data(1:t_end,i);
    
    indices = find(trace{trace_count}.signal{signal_count}==-1);
    if ~isempty(indices), 
        disp('error, -1 found, tracing not reliable?'); 
        disp(['number of -1 is ',num2str(length(indices))]);
        for m=indices
            dir = 0;
            if m==1, dir = 2; end;
            trace{trace_count}.signal{signal_count}(m) = (trace{trace_count}.signal{signal_count}(m-1+dir)+trace{trace_count}.signal{signal_count}(m+1))/2;
        end
    end
    
    
    signal_count = signal_count+1;
end
time = sample_i;

ref1 = trace{1}.signal{1};
ref2 = trace{1}.signal{2};
u1 = trace{2}.signal{1};
u2 = trace{2}.signal{2};
enc1 = trace{3}.signal{1};
enc2 = trace{3}.signal{2};
err1 = ref1-enc1;
err2 = ref2-enc2;

ffwm1 = trace{4}.signal{1};
ffwm2 = trace{4}.signal{2};
ffwa1 = trace{5}.signal{1};
ffwa2 = trace{5}.signal{2};


ref = [ref1,ref2];
u = [u1,u2];
enc = [enc1,enc2];
ffwm = [ffwm1,ffwm2];
ffwa = [ffwa1,ffwa2];

uc = u-ffwm-ffwa;
err = ref-enc;

% tr1 = trace{4}.signal{1};
% tr2 = trace{4}.signal{2};

%% plot results joint 1
n_plots = 3; i_p = 1; 
figure; scr rt;
ax(joint,i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
plot(time,ref(:,joint)); ylabel('ref [m]'); grid on; hold all;
plot(time,enc(:,joint)); legend('ref','enc'); title(['joint ',num2str(joint)]);
ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,uc(:,joint)); ylabel('control [Nm]'); grid on;  hold all;
plot(time,ffwm(:,joint));
plot(time,ffwa(:,joint));
plot(time,u(:,joint)); legend('PID','mod','acc','C')
ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,err(:,joint)); ylabel('err [m]'); grid on;
linkaxes(ax(joint,:),'x');




all_ylims_on();
all_xlims_on();
all_grids_on();

%% save
savedir= 'Data_acc/';

e = err(:,joint);
save([savedir,filename],'e','time');

