clear all; 
close all; 
clc;
plotsettings
%% on sergio
% filedir = '/home/amigo/ros/data/private/Ton_data/torso_validation/';
%% on my pc
filedir = '~/ros/data/private/Ton_data/base/ffw/';


% filename = 'Trunk_acc08_leg10';
% joint =2;
filename = 'FFW_xy_04';
joint =2;

data = importdata([filedir,filename,'.dat']);
vectorsizes = [3,3,3,3];
t_start = 0;
t_end = length(data.data(:,1));

WHEELRAD = 0.075*1.0147;
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
        disp(['number of -1 is ',num2str(length(indices)),', in signal ',num2str(trace_count)]);
        for mm=1:length(indices);
            m = indices(mm);
            dir = 0; dir2 = 0;
            if m==1, dir = 2; end;
            if m==length(trace{trace_count}.signal{signal_count}), dir2 = -2; end;
            trace{trace_count}.signal{signal_count}(m) = (trace{trace_count}.signal{signal_count}(m-1+dir)+trace{trace_count}.signal{signal_count}(m+1+dir2))/2;
        end
    end
    
    
    signal_count = signal_count+1;
end
time = sample_i;

ref1 = trace{1}.signal{1};
ref2 = trace{1}.signal{2};
ref3 = trace{1}.signal{3};
u1 = trace{2}.signal{1};
u2 = trace{2}.signal{2};
u3 = trace{2}.signal{3};
err1 = trace{3}.signal{1};
err2 = trace{3}.signal{2};
err3 = trace{3}.signal{3};


ffw1 = trace{4}.signal{1};
ffw2 = trace{4}.signal{2};
ffw3 = trace{4}.signal{3};


ref = [ref1,ref2,ref3];
u = [u1,u2,u3];
err = [err1,err2,err3];
ffw = [ffw1,ffw2,ffw3];

uc = u-ffw;

% tr1 = trace{4}.signal{1};
% tr2 = trace{4}.signal{2};
figure; scr lt;
plot(time,cumsum(err(:,joint))./1000);
%% plot results joint 1
n_plots = 3; i_p = 1; 
figure; scr rt;
ax(joint,i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
plot(time,ref(:,joint)); ylabel('ref [m]'); grid on; hold all;
% plot(time,enc(:,joint)); legend('ref','enc'); title(['joint ',num2str(joint)]);
ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,uc(:,joint)); ylabel('control [N]'); grid on;  hold all;
plot(time,ffw(:,joint));
plot(time,u(:,joint)); legend('PD','ffw','C')
ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,err(:,joint)); ylabel('err [m]'); grid on;
linkaxes(ax(joint,:),'x');




all_ylims_on();
all_xlims_on();
all_grids_on();

%% save
savedir= 'Data/';


save([savedir,filename],'time','ref','u','err','ffw','uc');

