% clear all; 
% close all; 
clc;

filedir = '/home/amigo/ros/data/private/Ton_data/torso_identification/';
filename = 'manual_ref07';

data = importdata([filedir,filename,'.dat']);
vectorsizes = [2,2,2];

GEARRATIO 			= 9.0/169.0;		
BITS2CURRENT 		= 25.0/2046.0; 		
CURRENT2TORQUE 		= 29.2e-3; 			
BITS2WHEELTORQUE 	= BITS2CURRENT * CURRENT2TORQUE * 1.0/GEARRATIO;
ENC2RAD = 2.0*3.141592*9.0/169.0/(500.0*4.0);

sample_i = data.data(:,1);
trace_count = 1;
signal_count = 1;
for i=2:1:sum(vectorsizes)+1
    if signal_count > vectorsizes(trace_count)
        trace_count = trace_count+1;
        signal_count = 1;
    end
    trace{trace_count}.signal{signal_count} = data.data(:,i);
    
    indices = find(trace{trace_count}.signal{signal_count}==-1);
    if ~isempty(indices), 
        disp('error, -1 found, tracing not reliable?'); 
        disp(['number of -1 is ',num2str(length(indices))]);
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


%% plot results
figure; 
subplot(3,1,1);
plot(time,ref1,time,ref2); ylabel('ref'); grid on; 
subplot(3,1,2);
plot(time,u1,time,u2); ylabel('control'); grid on;
subplot(3,1,3);
plot(time,enc1,time,enc2); ylabel('enc'); grid on;
linkaxes(get(gcf,'children'),'x');
