clear all; close all; clc;
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
            else
                H2{count} = H./(K_elm*K_m);
                hz2{count} = hz;
                Coh_S2{count} = Coh_du;
                Coh_PS2{count} = Coh_de;
            end
        end
        count = count+1;
    end
end
clearvars -EXCEPT H1 hz1 H2 hz2 Coh_S1 Coh_S2 Coh_PS2 Coh_PS1
plotsettings
%% load model sys
% load '/home/ton/Dropbox/Linux/Matlab/Torso/Model/sys_volt_spindle'
h1 = figure; scr lt;
h2 = figure; scr rt;
leg_list1 = {};leg_count1 = 1;
leg_list2 = {};leg_count2 = 1;
H_sum1 = zeros(size(H1{1}));
H_sum2 = zeros(size(H2{1}));
figC1 = figure; scr lb;
figC2= figure;scr rb;
for i=1:1:9
    % plot bode legs
    figure(h1);
    if i~=2 && i~=3
        ax1(1) = subplot(2,1,1);
        semilogx(hz1{i},db(H1{i}),'color',ps.list5{i}); hold all; 
        ylabel('Magnitude [db]');
        ax1(2) = subplot(2,1,2);
        semilogx(hz1{i},angle(H1{i}).*360/2/pi,'color',ps.list5{i}); hold all; 
        ylabel('Phase [degrees]'); xlabel('Frequency [Hz]');
        leg_list1{leg_count1} = num2str(i);leg_count1 = leg_count1+1;
        
        figure(figC1);
        axC1(1) = subplot(2,1,1);
        semilogx(hz1{i},abs(Coh_S1{i}),'color',ps.list5{i}); hold all; 
        ylabel('Coherens S');
        axC1(2) = subplot(2,1,2);
        semilogx(hz1{i},abs(Coh_PS1{i}),'color',ps.list5{i}); hold all; 
        ylabel('Coherens PS');
        xlabel('Frequency [Hz]');
        
        H_sum1 = H_sum1+H1{i};
    end
    
    % plot bode trunk
    figure(h2);
    ax2(1) = subplot(2,1,1);
    pl(i) = semilogx(hz2{i},db(H2{i}),'color',ps.list5{i}); hold all;
    ylabel('Magnitude [db]');
    ax2(2) = subplot(2,1,2);
    semilogx(hz2{i},angle(H2{i}).*360/2/pi,'color',ps.list5{i}); hold all;
    ylabel('Phase [degrees]'); xlabel('Frequency [Hz]');
    leg_list2{leg_count2} =  num2str(i); leg_count2 = leg_count2+1;
    
    figure(figC2);
    axC2(1) = subplot(2,1,1);
    semilogx(hz2{i},abs(Coh_S2{i}),'color',ps.list5{i}); hold all; 
    ylabel('Coherens S');
    axC2(2) = subplot(2,1,2);
    semilogx(hz2{i},abs(Coh_PS2{i}),'color',ps.list5{i}); hold all; 
    ylabel('Coherens PS');
    xlabel('Frequency [Hz]');
        
    H_sum2 = H_sum2+H2{i};
    
    all_grids_on();
end


linkaxes(ax1,'x');
linkaxes(ax2,'x');
linkaxes(axC1,'x');
linkaxes(axC2,'x');
set(ax1(1),'xticklabel',[]);
set(ax2(1),'xticklabel',[]);
set(axC1(1),'xticklabel',[]);
set(axC2(1),'xticklabel',[]);
axes(ax2(2));
legend(leg_list2);


all_grids_on();
all_ylims_on();
all_xlims_on();


Havg1 = H_sum1./7;
hzavg1 = hz1{1};
Havg2 = H_sum2./9;
hzavg2 = hz2{1};
% figure



legend(leg_list2);



