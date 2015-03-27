clear all; 
% close all; 
clc;
%%%% compare different measured frfs for wheel 1
% 1 in the air with ref vel
% 2 on the floor with ref vel
% 3 on the floor no ref vel

data{1} = load('frf_wheel1_air.mat');
data{2} = load('frf_wheel1_floor_v1.mat');
data{3} = load('frf_wheel1_floor_v0.mat');
plotsettings

fig1 = figure; scr r;
fig2 = figure; scr lt;
fig3 = figure; scr lb;
for i=1:1:3
    hz = data{i}.hz;
    H = data{i}.H;
    indices = find(hz>0.8);
%     if i==2
%         indices = find(hz>7);
%     end
    hz = hz(indices);
    H = H(indices);

    figure(fig1)
    subplot(4,1,1); 
    semilogx(hz,db(H),'color',ps.list3{i}); grid on;
    ylabel('Magnitude dB'); hold all;
    subplot(4,1,2); 
    semilogx(hz,angle(H).*(360/2/pi),'color',ps.list3{i});grid on;
    ylabel('Phase degrees'); xlabel('Frequency [Hz]');hold all;
    subplot(4,1,3); 
    semilogx(data{i}.hz,db(data{i}.S),'color',ps.list3{i}); grid on;
    ylabel('Sensitivity'); hold all;
    semilogx(data{i}.hz,db(data{i}.PS),'color',ps.list3{i});
    legend('S d-u','PS d-e');
    subplot(4,1,4); 
    semilogx(data{i}.hz,(data{i}.Scoh),'--','color',ps.list3{i}); grid on; ylim([0 1]); hold all;
    semilogx(data{i}.hz,(data{i}.PScoh),'color',ps.list3{i}); 
    ylabel('Coherene'); title('coherence'); 
    
    figure(fig2)
    Hfrd = frd(H,hz,'FrequencyUnit','Hz');
    Ci = C_integrator(0.1,1);
    Cll = C_leadlag(10,100);
    Clp = C_lowpass(500,2,0.7);
    C = 20*Ci*Cll*Clp;
    HCfrd = Hfrd*C;
    HC = squeeze(HCfrd.resp);
    subplot(2,1,1); 
    semilogx(hz,db(HC),'color',ps.list3{i}); grid on;
    ylabel('Magnitude dB'); hold all;
    subplot(2,1,2); 
    semilogx(hz,angle(HC).*(360/2/pi),'color',ps.list3{i});grid on;
    ylabel('Phase degrees'); xlabel('Frequency [Hz]');hold all;
    figure(fig3)
    plot(real(HC),imag(HC),'color',ps.list3{i}); hold all; grid on;
    
end
figure(fig3)
ylim([-3 3]); xlim([-4 2]);
tau = linspace(-pi,pi,100); im = 1/invdb(6)*sin(tau); re = 1/invdb(6)*cos(tau)-1;
plot(re,im,'r--',sin(tau),cos(tau),'k--');
figure(fig1)
subplot(4,1,1)
legend('air','floor 1vel','floor 0vel');

linkaxes(get(gcf,'children'),'x')