clear all; 
close all; 
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compare measured frf to model %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

K_m     = 29.2e-3;      % Nm/A,         Motor torque constant
K_elm   = 10;           % A/V_input,    Gain from input Voltage to Current     
plotsettings
set(0,'defaulttextinterpreter','latex');
%% load measured frfs
H1 = {};
hz1= {};
H2 = {};
hz2 = {};
Coh_S1 = {};
Coh_PS1 = {};
Coh_S2 = {};
Coh_PS2 = {};
count = 1;
parts = {'leg','tr'};
pos = {'do','ce','up'};
posl = {'min','cen','max'};
leg_list1= {}; leg_count1 = 1;
leg_list2= {}; leg_count2 = 1;
for p1 = [1 2 3]
    for p2 = [1 2 3]
        for part = [1 2]
            if (part==1 && p1==1 && p2 >1)
                continue
            end
            filename = ['06-02-15_FRF_',parts{part},'_',pos{p1},'_',pos{p2}];
            load(['frf/',filename]);
            if part==1
                H1{count} = H./(K_elm*K_m);
                hz1{count} = hz;
                Coh_S1{count} = Coh_du;
                Coh_PS1{count} = Coh_de;
                leg_list1{leg_count1} =  [posl{p1},' ',posl{p2}];  leg_count1= leg_count1+1;
            else
                H2{count} = H./(K_elm*K_m);
                hz2{count} = hz;
                Coh_S2{count} = Coh_du;
                Coh_PS2{count} = Coh_de;
                leg_list2{leg_count2} =  [posl{p1},' ',posl{p2}]; 
                leg_count2 = leg_count2+1;
            end
        end
        count = count+1;
    end
end
clearvars -EXCEPT H1 hz1 H2 hz2 Coh_S1 Coh_S2 Coh_PS2 Coh_PS1 leg_list2 leg_list1
plotsettings
%% load model sys
% load '/home/ton/Dropbox/Linux/Matlab/Torso/Model/sys_volt_spindle'
% h1 = figure; scr lt;
% h2 = figure; scr rt;
% leg_list1 = {};leg_count1 = 1;
% leg_list2 = {};leg_count2 = 1;
H_sum1 = zeros(size(H1{1}));
CS_sum1 = H_sum1;
CPS_sum1 = H_sum1;
H_sum2 = zeros(size(H2{1}));
CS_sum2 = H_sum2;
CPS_sum2 = H_sum2;
% figC1 = figure; scr lb;
% figC2= figure;scr rb;
for i=1:1:9
    % plot bode legs
%     figure(h1);
    if i~=2 && i~=3
%         ax1(1) = subplot(4,1,1);
%         semilogx(hz1{i},db(H1{i}),'color',ps.list5{i}); hold all; 
%         ylabel('Magnitude [db]');
%         ax1(2) = subplot(4,1,2);
%         semilogx(hz1{i},angle(H1{i}).*360/2/pi,'color',ps.list5{i}); hold all; 
%         ylabel('Phase [degrees]'); %xlabel('Frequency [Hz]');
% %         leg_list1{leg_count1} = num2str(i);leg_count1 = leg_count1+1;
%         
%         figure(h1);
%         ax1(3) = subplot(4,1,3);
%         semilogx(hz1{i},abs(Coh_S1{i}),'color',ps.list5{i}); hold all; 
%         ylabel('Coherens S');
%         ax1(4) = subplot(4,1,4);
%         semilogx(hz1{i},abs(Coh_PS1{i}),'color',ps.list5{i}); hold all; 
%         ylabel('Coherens PS');
%         xlabel('Frequency [Hz]');
        
        H_sum1 = H_sum1+H1{i};
        CS_sum1 = CS_sum1+Coh_S1{i};
        CPS_sum1 = CPS_sum1+Coh_PS1{i};
    end
    
    % plot bode trunk
%     figure(h2);
%     ax2(1) = subplot(4,1,1);
%     pl(i) = semilogx(hz2{i},db(H2{i}),'color',ps.list5{i}); hold all;
%     ylabel('Magnitude [db]');
%     ax2(2) = subplot(4,1,2);
%     semilogx(hz2{i},angle(H2{i}).*360/2/pi,'color',ps.list5{i}); hold all;
%     ylabel('Phase [degrees]'); %xlabel('Frequency [Hz]');
%     % coherence
%     ax2(3) = subplot(4,1,3);
%     semilogx(hz2{i},abs(Coh_S2{i}),'color',ps.list5{i}); hold all; 
%     ylabel('Coherens S');
%     ax2(4) = subplot(4,1,4);
%     semilogx(hz2{i},abs(Coh_PS2{i}),'color',ps.list5{i}); hold all; 
%     ylabel('Coherens PS');
%     xlabel('Frequency [Hz]');
        
    H_sum2 = H_sum2+H2{i};
    CS_sum2 = CS_sum2+Coh_S2{i};
    CPS_sum2 = CPS_sum2+Coh_PS2{i};
    
end
Havg1 = H_sum1./7;
CSavg1 = CS_sum1./7;
CPSavg1 = CPS_sum1./7;
hzavg1 = hz1{1};
Havg2 = H_sum2./9;
CSavg2 = CS_sum2./9;
CPSavg2 = CPS_sum2./9;
hzavg2 = hz2{1};

hz1 = hzavg1;
Hfrd1 = frd(Havg1,hz1,'frequencyunit','Hz');
hz2 = hzavg2;
Hfrd2 = frd(Havg2,hz2,'frequencyunit','Hz');

%% Controllers
% Cll = C_leadlag(3.33, 30);
% Ci = C_integrator(2.0, 1);
% C1 = 300*Cll*Ci;
% C2 = 330*Cll*Ci;

Cll = C_leadlag(3.33, 40);
Ci = C_integrator(2.0, 1);
Clp = C_lowpass(200,2,0.7);
C1 = 250*Cll*Ci*Clp;
C2 = 280*Cll*Ci*Clp;

CHfrd1 = C1*Hfrd1;
CH1 = squeeze(CHfrd1.resp);
CHfrd2 = C2*Hfrd2;
CH2 = squeeze(CHfrd2.resp);

indi =find(hz1>3.4);

fig1 = figure; scr rt;
plot(real(CH2),imag(CH2)); hold all;
plot(real(CH1(indi)),imag(CH1(indi))); 
xlabel('Real'); ylabel('Imaginary'); set(gcf,'numbertitle','off','name','Bode diag');
hold all; 
tau = linspace(-pi,pi,100); im = 1/invdb(6)*sin(tau); re = 1/invdb(6)*cos(tau)-1;
plot(re,im,'-k');%,'color',ps.tuewarmred);
plot(sin(tau),cos(tau),'--k');%,'color',ps.tuewarmred);
leg = legend('$Trunk$','$Legs$','location','northwest');
set(leg,'interpreter','latex')
set(gca,'ylim',[-3.5 1.5]); 
set(gca,'xlim',[-3.5 1.5]);


fig2 = figure; scr lt;
ax(1) = subplot(2,1,1);
semilogx(hz1,db(CH2)); hold all; 
semilogx(hz2,db(CH1));
ylabel('Magnitude [dB]'); 
ax(2) = subplot(2,1,2); 
semilogx(hz1,angle(CH2).*(360/2/pi)); hold all;
semilogx(hz2,angle(CH1).*(360/2/pi));
ylabel('Phase [degr.]'); hold all;
xlabel('Frequency [Hz]');

linkaxes(ax,'x');
set(ax(1),'xticklabel',[]);
set_xlim(ax(1));
set_ylim(ax(1));
set_ylim(ax(2));
all_grids_on();
% all_ylims_on();
% all_xlims_on();

%% print results
pmar1=  phasemargin(CH1(indi));
pmar2 = phasemargin(CH2);

S1 = (1/(1+CHfrd1));
mmar1 = max(db(squeeze(S1.resp)));
S2 = (1/(1+CHfrd2));
mmar2 = max(db(squeeze(S2.resp)));

ibw1 = find(db(CH1(indi))<0,1,'first')+indi(1)-1;
bw1 = interp1(db(CH1(ibw1-1:ibw1)),hz1(ibw1-1:ibw1),0);

ibw2 = find(db(CH2)<0,1,'first');
bw2 = interp1(db(CH2(ibw2-1:ibw2)),hz2(ibw2-1:ibw2),0);




disp(['Legs  bandwidth = ',num2str(bw1,4),', phase = ',num2str(pmar1,3),', modulus = ',num2str(mmar1,3)]);
disp(['Trunk bandwidth = ',num2str(bw2,4),', phase = ',num2str(pmar2,3),', modulus = ',num2str(mmar2,3)]);

