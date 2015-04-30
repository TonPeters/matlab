clear all; 
close all; 
clc;

%% on sergio
% filedir = '/home/amigo/ros/data/private/Ton_data/torso_identification/frf_measurements/';
%% on my pc
filedir = '/home/ton/ros/data/private/Ton_data/torso_identification/frf_measurements/';

% part = 2;
% p1 = 3; 
% p2 =3;
f{1} = figure;
h1 = figure; scr rb;
part2_first = true;
for part = [1 2]
    if part==2&& part2_first
        f{2} = figure; scr rt; part2_first = false;
    end
    for p1 = [1 2 3]
        for p2 = [1 2 3]
            if (part==1 && p1==1 && p2 >1)
                continue
            end
            
parts = {'leg','tr'};
pos = {'do','ce','up'};
filename = ['06-02-15_FRF_',parts{part},'_',pos{p1},'_',pos{p2}];
joint = part;
naverage = 50;
fs = 1000;
EQ_LEG = [0.2261, 0.26,   0.26;   0.1761, 0.1905, 0.2109; 0.1206, 0.1352, 0.1667];
EQ_TRU = [0.0874, 0.0529, 0.0170; 0.0997, 0.0723, 0.0417; 0.1085, 0.0889, 0.0683];
ff = [EQ_LEG(p1,p2),EQ_TRU(p1,p2)];

%% import data
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
        for m=indices
%             trace{trace_count}.signal{signal_count}(m) = (trace{trace_count}.signal{signal_count}(m-1)+trace{trace_count}.signal{signal_count}(m+1))/2;
        end
    end
    
    
    signal_count = signal_count+1;
end
time = sample_i;

u1 = trace{1}.signal{1}-ff(1);
u2 = trace{1}.signal{2}-ff(2);
c1 = trace{2}.signal{1};
c2 = trace{2}.signal{2};
err1 = trace{3}.signal{1};
err2 = trace{3}.signal{2};
d1 = u1-c1;
d2 = u2-c2;

%% plot results
n_plots = 4; i_p = 1;
% figure; 
% subplot(n_plots,1,i_p); i_p = i_p+1;
% plot(time,err1,time,err2); ylabel('err [m]'); grid on; 
% subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,u1,time,u2); ylabel('control [V]'); grid on; 
% subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,d1,time,d2); ylabel('d [m]'); grid on;
% subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,add1,time,add2); ylabel('input [m]'); grid on;
% linkaxes(get(gcf,'children'),'x');

%% 
err = [err1,err2];
dd = [d1,d2];
cc = [c1,c2];
uu = [u1,u2];

%% create SISO frf
ei = err(:,joint);
di = dd(:,joint);
ui = uu(:,joint);

shift = 0;
if (part==2 && p1==1 && p2==2), shift = 1; end;
cut_start = 2;
cut_end = 2;
e = ei(cut_start+shift:end-cut_end);
u = ui(cut_start:end-cut_end-shift);
d = di(cut_start:end-cut_end-shift);
%     lp = lpi(1+cut_start:end-cut_end-shift);

% Average
nfft=round(length(e)/naverage);
window=hann(nfft);
noverlap=round(nfft*0.5);

[TrS,hz]=tfestimate(d,u,window,noverlap,nfft,fs);
[Coh_S,hz]=mscohere(d,u,window,noverlap,nfft,fs);
[TrPS,hz]=tfestimate(d,e,window,noverlap,nfft,fs);
[Coh_PS,hz]=mscohere(d,e,window,noverlap,nfft,fs);

H = -TrPS./(TrS);
Hfrd = frd(H,hz,'frequencyunit','Hz');

%% plot results
% create figures
% h3 = figure; %scr lt;
% h1 = figure; %scr rt;
% h2 = figure; %scr rb;

% plot data
% figure
figure(f{part});
subplot(2,1,1);
semilogx(hz,db(H)); hold all;
subplot(2,1,2);
semilogx(hz,angle(H).*360/2/pi); hold all;

figure(h1)
subplot(2,1,1)
semilogx(hz,Coh_S); ylabel('coh sens');
hold all;
subplot(2,1,2)
semilogx(hz,Coh_PS); ylabel('coh proc sens');
hold all;

% figure settings
% figure(gcf); grid on; title('Coherence d to u'); xlabel('frequency [hz]');
% ylabel('magnitude abs'); 
% figure(h2); grid on; title('Coherence d to e'); xlabel('frequency [hz]');
% ylabel('magnitude abs');      
figure(f{part});
subplot(2,1,1); grid on; ylabel('Magnitude db');
subplot(2,1,2); grid on; ylabel('angle degrees'); xlabel('frequency Hz');
%% save frf
% err = [err1,err2];
% dd = [d1,d2];
% cc = [c1,c2];
% uu = [u1,u2];
% save(['frf/',filename],'H','hz','err','dd','cc','uu','Coh_du','Coh_de');

all_grids_on();
        end
    end
end