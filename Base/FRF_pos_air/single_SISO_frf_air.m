% create single frf
clear all; 
close all; 
clc;

%% select PC
% on sergio
% filedir = '/home/amigo/ros/data/private/Ton_data/base/frf_air_pos/';
% on my pc
filedir = '/home/ton/ros/data/private/Ton_data/base/frf_air_pos/';

%% select file and wheel
% filename = ['19-03-15_FRF_pos_wheel1'];
% filename = ['20-03-15_FRF_pos_wheel1_noise06_vel60'];
% filename = ['20-03-15_FRF_pos_wheel1_noise06_vel60_newC'];

for wheel = 1 %[1,2,3,4]

filename = ['final/20-03-15_FRF_pos_wheel',num2str(wheel)];





data = importdata([filedir,filename,'.dat']);
vectorsizes = [4, 4, 4];


WHEELRAD 			= 0.075*1.0147; %	# Corrected for effective wheel radius
HALFSQRT2 			= 0.7071;%
GEARRATIO 			= 9.0/169.0;	%	# Gearratio Maxxon GP42C
BITS2CURRENT 		= 50.0/2046.0; 	%	# Bit value to current value (+-2046 bit FPGA-> +-5V ELMO-> +-50A)
CURRENT2TORQUE 		= 29.2e-3; 		%	# Torque constant of the motor Maxxon RE35
BITS2TORQUE         = BITS2CURRENT * CURRENT2TORQUE;
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
cut_start = 4000;
cut_end = 0;
shift = 0;
time = sample_i(cut_start:end-shift);
u = trace{1}.signal{wheel}(cut_start:end-shift);
c = trace{2}.signal{wheel}(cut_start:end-shift);
e = trace{3}.signal{wheel}(cut_start+shift:end);
d = u-c;

% figure; scr r; 
% subplot(3,1,1)
% plot(time,e); grid on; ylabel('e');
% subplot(3,1,2)
% plot(time,d); grid on;ylabel('d [bits]');
% subplot(3,1,3)
% plot(time,u); grid on;ylabel('u [bits]');
% linkaxes(get(gcf,'children'),'x')

% create frf
% frf settings
fs=1000;
naverage=50;
nfft=round(length(e)/naverage);
% nfft=1024;
window=hann(nfft);
noverlap=round(nfft*0.5);

[S,hz]=tfestimate(d,u,window,noverlap,nfft,fs);
[Scoh,hz]=mscohere(d,u,window,noverlap,nfft,fs);
[PS,hz]=tfestimate(d,e,window,noverlap,nfft,fs);
[PScoh,hz]=mscohere(d,e,window,noverlap,nfft,fs);

H=-PS./S;


figure; scr r;
subplot(4,1,1); 
semilogx(hz,db(H)); grid on;
ylabel('Magnitude dB'); hold all;
subplot(4,1,2); 
semilogx(hz,angle(H).*(360/2/pi));grid on;
ylabel('Phase degrees'); xlabel('Frequency [Hz]');hold all;
subplot(4,1,3); 
semilogx(hz,(Scoh),hz,(PScoh)); grid on; ylim([0 1])
ylabel('Magnitude'); title('coherence'); legend('d-u','d-e');
subplot(4,1,4); 
semilogx(hz,db(S),hz,db(PS)); grid on;
ylabel('Magnitude dB'); hold all;
linkaxes(get(gcf,'children'),'x')

end
%% controller
% close all
% gain = 1.0;
% Clp = C_lowpass(500,2);
% Cll = C_leadlag(1.7, 5);
% 
% C = gain*Clp*Cll;
% Hfrd = frd(H,hz,'units','hz');
% 
% CP = C*Hfrd;
% figure
% frf(Hfrd,CP,C)

%% save data
% save(['frf_wheel',num2str(wheel),'_air.mat'],'H','hz','S','PS','Scoh','PScoh');