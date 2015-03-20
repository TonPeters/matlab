% create single frf
clear all; close all; clc;

%% on sergio
filedir = '/home/amigo/ros/data/private/Ton_data/base/frf_air_pos/';
%% on my pc
% filedir = '/home/ton/ros/data/private/Ton_data/torso_identification/frf_measurements/';

filename = ['19-03-15_FRF_pos_wheel1'];
wheel = 1;


data = importdata([filedir,filename,'.dat']);
vectorsizes = [4, 4, 4];


WHEELRAD 			= 0.075*1.0147; %	# Corrected for effective wheel radius
HALFSQRT2 			= 0.7071;%
GEARRATIO 			= 9.0/169.0;	%	# Gearratio Maxxon GP42C
BITS2CURRENT 		= 50.0/2046.0; 	%	# Bit value to current value (+-2046 bit FPGA-> +-5V ELMO-> +-50A)
CURRENT2TORQUE 		= 29.2e-3; 		%	# Torque constant of the motor Maxxon RE35
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
        disp(['occurences of -1 is ',num2str(length(indices))]);
    end
    indices_list{i} = indices;
    signal_count = signal_count+1;
end

%%
cut_start = 1000;
cut_end = 0;
shift = 1;
time = sample_i(cut_start:end-shift);
u = trace{1}.signal{wheel}(cut_start:end-shift);
c = trace{2}.signal{wheel}(cut_start:end-shift);
e = trace{3}.signal{wheel}(cut_start+shift:end);
d = u-c;

figure; scr r; 
subplot(3,1,1)
plot(time,e); grid on; ylabel('e');
subplot(3,1,2)
plot(time,d./BITS2WHEELTORQUE); grid on;ylabel('d');
subplot(3,1,3)
plot(time,u./BITS2WHEELTORQUE); grid on;ylabel('u');
linkaxes(get(gcf,'children'),'x')

% create frf
% frf settings
fs=1000;
naverage=50;
nfft=round(length(e)/naverage);
% nfft=1024;
window=hann(nfft);
noverlap=round(nfft*0.5);

[TrS,hz]=tfestimate(d,u,window,noverlap,nfft,fs);
[Trdu,hz]=mscohere(d,u,window,noverlap,nfft,fs);
[TrPS,hz]=tfestimate(d,e,window,noverlap,nfft,fs);
[Trde,hz]=mscohere(d,e,window,noverlap,nfft,fs);

H=-TrPS./TrS;

figure; scr r;
semilogx(hz,db(TrPS)); grid on;
ylabel('Magnitude dB'); hold all;


figure; scr r;
subplot(4,1,1); 
semilogx(hz,db(H)); grid on;
ylabel('Magnitude dB'); hold all;
subplot(4,1,2); 
semilogx(hz,angle(H).*(360/2/pi));grid on;
ylabel('Phase degrees'); xlabel('Frequency [Hz]');hold all;
subplot(4,1,3); 
semilogx(hz,(Trdu),hz,(Trde)); grid on;
ylabel('Magnitude'); title('coherence'); legend('d-u','d-e');
subplot(4,1,4); 
semilogx(hz,db(TrPS),hz,db(TrS)); grid on;
ylabel('Magnitude dB'); hold all;
linkaxes(get(gcf,'children'),'x')

% save(['frf_wheel',num2str(wheel),'_air.mat'],'H','hz');