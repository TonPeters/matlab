clear all; 
close all; 
clc;

%% on sergio
filedir = '/home/amigo/ros/data/private/Ton_data/torso_identification/friction_trunk/';
%% on my pc
% filedir = '/home/ton/ros/data/private/Ton_data/torso_identification/';
fig1 = figure;
% fil = {'18','20','22','24','26'}; %'04','06','08','10','12','14','16',
fil = {'06','24'};
for k=1:1:length(fil)
filename = ['m10kg_vel0',fil{k}];

data = importdata([filedir,filename,'.dat']);
vectorsizes = [2,2,2,2];

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
ffw1 = trace{4}.signal{1};
ffw2 = trace{4}.signal{2};

% tr1 = trace{4}.signal{1};
% tr2 = trace{4}.signal{2};

%% plot results
n_plots = 3; i_p = 1;
figure; 
subplot(n_plots,1,i_p); i_p = i_p+1;
plot(time,ref1,time,ref2); ylabel('ref [m]'); grid on; hold all;
plot(time,enc1,time,enc2); legend('ref1','ref2','enc1','enc2');
subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,u2,time,ffw2); ylabel('control [V]'); grid on; 
% subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,enc1,time,enc2); ylabel('enc [m]'); grid on;
subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,err1,time,err2); ylabel('err [m]'); grid on;
linkaxes(get(gcf,'children'),'x');

%% plot reference
% figure; 
% subplot(3,1,1); plot(time,ref1,time,ref2); ylabel('ref m'); 
% subplot(3,1,2); plot_tdiff(time,ref1); hold all;
% plot_tdiff(time,ref2); ylabel('ref vel m/s');
% subplot(3,1,3); plot(time(1:end-2),diff(diff(ref1).*1000));hold all;
% plot(time(1:end-2),diff(diff(ref2).*1000)); ylabel('ref acc m/s2'); xlabel('time s');
% all_grids_on(); linkaxes(get(gcf,'children'),'x');
%%
indices = find(diff(ref2)>0);
figure(fig1);
plot(enc2(indices),u2(indices)); grid on; hold all
xlabel('enc2 m'); ylabel('input1 V');
indices = find(diff(ref2)<0);
plot(enc2(indices),u2(indices));
% figure;
% plot(enc1,u1); grid on;
% xlabel('enc2 m'); ylabel('input2 V');
%% save data
% ref = [ref1, ref2];
% enc = [enc1, enc2];
% u = [u1, u2];
% 
% save(['../data/friction_trunk/',filename,'.mat'],'ref','enc','u','time')

end
