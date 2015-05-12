clear all;
close all; 
clc;
% fig1 = figure;scr r;
%% Choose PC
% on sergio
filedir = '/home/amigo/ros/data/private/Ton_data/torso_identification/identification_leg/';
% on my pc
% filedir = '/home/ton/ros/data/private/Ton_data/torso_identification/identification_leg/';
%%

filename = 'm0kg_vel004_trunk22';

data = importdata([filedir,filename,'.dat']);
vectorsizes = [2,2,2,2];
t_start = 0;
t_end = length(data.data(:,1));

GEARRATIO 			= 9.0/169.0;		
BITS2CURRENT 		= 25.0/2046.0; 		
CURRENT2TORQUE 		= 29.2e-3; 			
BITS2WHEELTORQUE 	= BITS2CURRENT * CURRENT2TORQUE * 1.0/GEARRATIO;
ENC2RAD = 2.0*3.141592*9.0/169.0/(500.0*4.0);
K_m     = 29.2e-3;      % Nm/A,         Motor torque constant
K_elm   = 10;           % A/V_input,    Gain from input Voltage to Current     

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
ffw1 = trace{4}.signal{1};
ffw2 = trace{4}.signal{2};
% ffw2 = zeros(size(ref1));

% tr1 = trace{4}.signal{1};
% tr2 = trace{4}.signal{2};


%% plot results
n_plots = 3; i_p = 1;
figure; 
subplot(n_plots,1,i_p); i_p = i_p+1;
plot(time,ref1); ylabel('ref [m]'); grid on; hold all;
plot(time,enc1); legend('ref2','enc2');
% plot(time,ffw2); hold all;
subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,u1.*(K_elm*K_m)); ylabel('control [Nm]'); grid on;  hold all;
% plot(time,ffw2);
% subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,enc1,time,enc2); ylabel('enc [m]'); grid on;
subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,err1); ylabel('err [m]'); grid on;
linkaxes(get(gcf,'children'),'x');

%% plot results
% n_plots = 3; i_p = 1;
% figure(fig1); 
% subplot(n_plots,1,i_p); i_p = i_p+1;
% plot_diff(time,ref1); ylabel('ref [m/s]'); grid on; hold all;
% % plot_tdiff(time,enc1);
% subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,u1); ylabel('control [V]'); grid on;  hold all;
% % plot(time,ffw2);
% % subplot(n_plots,1,i_p);i_p = i_p+1;
% % plot(time,enc1,time,enc2); ylabel('enc [m]'); grid on;
% subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,err1); ylabel('err [m]'); grid on; hold all;
% % linkaxes(get(gcf,'children'),'x');


%% plot error peaks
% duration = 6000;
% irem = find(diff(enc2)<-2e-6,1,'first');
% ist = find(diff(enc2(irem+duration-1000:end))>2e-6,1,'first')+irem+duration-1000-150;
% iend = ist;
% fig1 = figure;
% while (iend<length(time))
%     iend = ist+2*duration;
%     plend = iend-duration;
%     if plend>length(time), plend = length(time); end;
%     figure(fig1); subplot(3,1,1);
%     plot(enc2(ist:plend),err2(ist:plend)); hold all; ylabel('err [m]')
%      subplot(3,1,2);
%     plot(enc2(ist:plend),u2(ist:plend)+ffw2(ist:plend)); hold all;ylabel('u_{fb} [V]')
%      subplot(3,1,3);
%     plot(enc2(ist:plend),diff(enc2((ist-1:plend)))); hold all;ylabel('ref [m/s]')
% %     plot_tdiff(time(ist:plend)-time(ist),enc2(ist:plend)); hold all;
%     ist = iend;
% end
% linkaxes(get(gcf,'children'),'x');
% 
% all_grids_on();

%% plot input
% res = 5/1023;
% figure
% subplot(2,1,1)
% plot(time,ffw2+u2,time,ffw2,time,u2,time,ones(size(time)).*res,'y',time,-ones(size(time)).*res,'y'); 
% grid on; legend('u_{total}','u_{ffw}','u_{fb}','+res','-res')
% subplot(2,1,2)
% plot(time,round((ffw2+u2)./res).*res,time,round(ffw2./res).*res,time,round(u2./res).*res); 
% linkaxes(get(gcf,'children'),'x');


%% plot

% figure
% plot(diff(enc2(1000:2000)),u2(1000:2000-1));
% 
% all_grids_on();


%% plot reference
% figure; 
% subplot(3,1,1); plot(time,ref1,time,ref2); ylabel('ref m'); 
% subplot(3,1,2); plot_tdiff(time,ref1); hold all;
% plot_tdiff(time,ref2); ylabel('ref vel m/s');
% subplot(3,1,3); plot(time(1:end-2),diff(diff(ref1).*1000));hold all;
% plot(time(1:end-2),diff(diff(ref2).*1000)); ylabel('ref acc m/s2'); xlabel('time s');
% all_grids_on(); linkaxes(get(gcf,'children'),'x');
%%
figure
% figure(fig1);
plot(enc1,u1); grid on; hold all
xlabel('enc1 m'); ylabel('input1 V');
% figure;
% plot(enc1,u1); grid on;
% xlabel('enc2 m'); ylabel('input2 V');

%% cut off data
% indices.p1 = find(time>6 & time<34);
% indices.n1 = find(time>38 & time<70);
% indices.p2 = find(time>78 & time<106);
% indices.n2 = find(time>110 & time<142);
% 
% indic = {indices.p1, indices.n1, indices.p2, indices.n2};
% for i = 1:4
%     indi = indic{i};
%     n_plots = 3; i_p = 1;
%     figure(fig1); 
%     subplot(n_plots,1,i_p); i_p = i_p+1;
%     plot_diff(time(indi),ref1(indi)); ylabel('ref [m/s]'); grid on; hold all;
%     % plot_tdiff(time,enc1);
%     subplot(n_plots,1,i_p);i_p = i_p+1;
%     plot(time(indi),u1(indi)); ylabel('control [V]'); grid on;  hold all;
%     % plot(time,ffw2);
%     % subplot(n_plots,1,i_p);i_p = i_p+1;
%     % plot(time,enc1,time,enc2); ylabel('enc [m]'); grid on;
%     subplot(n_plots,1,i_p);i_p = i_p+1;
%     plot(time(indi),err1(indi)); ylabel('err [m]'); grid on;
%     % linkaxes(get(gcf,'children'),'x');
% end
%% save data
% ref = [ref1, ref2]; 
% enc = [enc1, enc2];
% u = [u1, u2];
% 
% save(['data_friction/',filename,'.mat'],'ref','enc','u','time','indices')

all_grids_on();
