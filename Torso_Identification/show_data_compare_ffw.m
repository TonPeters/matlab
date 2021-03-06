clear all; 
close all; 
clc;

%% on sergio
% filedir = '/home/amigo/ros/data/private/Ton_data/torso_identification/ffw_trunk/';
%% on my pc
filedir = '/home/ton/ros/data/private/Ton_data/torso_identification/ffw_trunk/';

filename{1} = 'test_noffw_trunk_complete_system';
filename{2} = 'test_ffw_trunk_complete_system';

fig1 = figure; scr r;
for k=1:1:2
data = importdata([filedir,filename{k},'.dat']);
vectorsizes = [2,2,2];
t_start = 0;
t_end = length(data.data(:,1));

GEARRATIO 			= 9.0/169.0;		
BITS2CURRENT 		= 25.0/2046.0; 		
CURRENT2TORQUE 		= 29.2e-3; 			
BITS2WHEELTORQUE 	= BITS2CURRENT * CURRENT2TORQUE * 1.0/GEARRATIO;
ENC2RAD = 2.0*3.141592*9.0/169.0/(500.0*4.0);
Kelm = 10;
facc = 0.8*0.024;

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
u1 = trace{2}.signal{1};
u2 = trace{2}.signal{2};
enc1 = trace{3}.signal{1};
enc2 = trace{3}.signal{2};
err1 = ref1-enc1;
err2 = ref2-enc2;
% ffw1 = trace{4}.signal{1};
% ffw2 = trace{4}.signal{2};
% ffw2 = zeros(size(ref1));

% tr1 = trace{4}.signal{1};
% tr2 = trace{4}.signal{2};

%% plot results
n_plots = 3; i_p = 1;
figure(fig1); 
subplot(n_plots,1,i_p); i_p = i_p+1;
plot(time,ref2); ylabel('ref [m]'); grid on; hold all;
plot(time,enc2); legend('ref2','enc2');
% plot(time,ffw2); hold all;
subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,u2); ylabel('control [V]'); grid on;  hold all;
% plot(time,ffw2);
% subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,enc1,time,enc2); ylabel('enc [m]'); grid on;
subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,err2); hold all;
ylabel('err [m]'); grid on;
linkaxes(get(gcf,'children'),'x');

end

all_grids_on();




