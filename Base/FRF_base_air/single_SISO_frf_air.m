% create single frf
clear all; close all; clc;

filename = '31-10-14_FRF_wheel_air';
data = importdata([filename,'.dat']);
vectorsizes = [4, 4, 4];
wheel = 1;

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
plot(time,d); grid on;ylabel('d');
subplot(3,1,3)
plot(time,u); grid on;ylabel('u');
linkaxes(get(gcf,'children'),'x')

% create frf
% frf settings
fs=1000;
naverage=5;
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
subplot(3,1,1); 
semilogx(hz,db(H)); grid on;
ylabel('Magnitude dB'); hold all;
subplot(3,1,2); 
semilogx(hz,angle(H).*(360/2/pi));grid on;
ylabel('Phase degrees'); xlabel('Frequency [Hz]');hold all;
subplot(3,1,3); 
semilogx(hz,(Trdu),hz,(Trde)); grid on;
ylabel('Magnitude'); title('coherence'); legend('d-u','d-e');

linkaxes(get(gcf,'children'),'x')

% save(['frf_wheel',num2str(wheel),'_air.mat'],'H','hz');