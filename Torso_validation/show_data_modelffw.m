clear all; 
close all; 
clc;
plotsettings
%% on sergio
% filedir = '/home/amigo/ros/data/private/Ton_data/torso_validation/';
%% on my pc
filedir = '/home/ton/ros/data/private/Ton_data/torso_validation/';


filename = 'stand_bend_PD_ffw';

data = importdata([filedir,filename,'.dat']);
vectorsizes = [2,2,2,2];
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
r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
l_ls    = 0.002;        % m,           Lead of the spindle
r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation
r_1 = r_gear1*r_sp;
r_2 = r_gear2*r_sp;
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
            trace{trace_count}.signal{signal_count}(m) = (trace{trace_count}.signal{signal_count}(m-1)+trace{trace_count}.signal{signal_count}(m+1))/2;
        end
    end
    
    
    signal_count = signal_count+1;
end
time = sample_i;

ref1 = trace{1}.signal{1};
ref2 = trace{1}.signal{2};
u1 = trace{2}.signal{1}.*r_1;
u2 = trace{2}.signal{2}.*r_2;
enc1 = trace{3}.signal{1};
enc2 = trace{3}.signal{2};
err1 = ref1-enc1;
err2 = ref2-enc2;

if strcmp(filename(end-2:end),'PID') || strcmp(filename(end-4:end-2),'PID')
    disp('no feedforward');
    ffw1 = zeros(size(u1));
    ffw2 = zeros(size(u2));
else
    disp('yes feedforward');
    ffw1 = trace{4}.signal{1}.*r_1;
    ffw2 = trace{4}.signal{2}.*r_2;
end

ref = [ref1,ref2];
u = [u1,u2];
enc = [enc1,enc2];
ffw = [ffw1,ffw2];

uc = u-ffw;
err = ref-enc;

% tr1 = trace{4}.signal{1};
% tr2 = trace{4}.signal{2};

%% plot results joint 1
n_plots = 3; i_p = 1; joint =1;
figure; scr rt;
ax(joint,i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
plot(time,ref(:,joint)); ylabel('ref [m]'); grid on; hold all;
plot(time,enc(:,joint)); legend('ref','enc'); title(['joint ',num2str(joint)]);
ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,uc(:,joint)); ylabel('control [Nm]'); grid on;  hold all;
plot(time,ffw(:,joint));
plot(time,u(:,joint)); legend('PID','ffw','C')
ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,err(:,joint)); ylabel('err [m]'); grid on;
linkaxes(ax(joint,:),'x');

%% plot results joint 2
n_plots = 3; i_p = 1; joint =2;
figure; scr lt;
ax(joint,i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
plot(time,ref(:,joint)); ylabel('ref [m]'); grid on; hold all;
plot(time,enc(:,joint)); legend('ref','enc'); title(['joint ',num2str(joint)]);
ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,uc(:,joint)); ylabel('control [Nm]'); grid on;  hold all;
plot(time,ffw(:,joint));
plot(time,u(:,joint)); legend('PID','ffw','C')
ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,err(:,joint)); ylabel('err [m]'); grid on;
linkaxes(ax(joint,:),'x');


all_ylims_on();
all_xlims_on();
all_grids_on();

%% save data
savedir = 'Data_model';

save([savedir,filename],'time','ref','u','enc','ffw','uc','err');