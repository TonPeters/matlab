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
h1 = figure; scr lt;
h2 = figure; scr rt;
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
    figure(h1);
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

figure(h1);
ax1(1) = subplot(3,1,1);
pl1(1) = semilogx(hzavg1,db(Havg1),'color',ps.list5{2}); hold all; 
% ylabel('Magnitude [db]');
title('Legs');
ax1(2) = subplot(3,1,2);
semilogx(hzavg1,angle(Havg1).*360/2/pi,'color',ps.list5{2}); hold all; 
% ylabel('Phase [degrees]');
ax1(3) = subplot(3,1,3);
pl1(2) = semilogx(hzavg1,abs(CSavg1),'color',ps.list5{3}); hold all; 
pl1(3) = semilogx(hzavg1,abs(CPSavg1),'color',ps.list5{4});
% ylabel('Coherens');
xlabel('Frequency [Hz]');

figure(h2);
ax2(1) = subplot(3,1,1);
pl2(1) = semilogx(hzavg2,db(Havg2),'color',ps.list5{2}); hold all; 
ylabel('Magnitude [db]'); 
title('Trunk');
ax2(2) = subplot(3,1,2);
semilogx(hzavg2,angle(Havg2).*360/2/pi,'color',ps.list5{2}); hold all; 
ylabel('Phase [deg.]');
ax2(3) = subplot(3,1,3);
pl2(2) = semilogx(hzavg2,abs(CSavg2),'color',ps.list5{3}); hold all; 
pl2(3) = semilogx(hzavg2,abs(CPSavg2),'color',ps.list5{4});
ylabel('Coherens');
xlabel('Frequency [Hz]');

linkaxes(ax1,'x');
linkaxes(ax2,'x');
set(ax1(1:2),'xticklabel',[]);
set(ax2(1:2),'xticklabel',[]);
axes(ax1(1));
leg1 = legend_outside(h1,{'Plant','Coherence $S$','Coherence $PS$'},pl1);
axes(ax2(1));
% leg2 = legend_outside(h2,{'Plant','Coherence $S$','Coherence $PS$'},pl2);




all_grids_on();
all_ylims_on();
all_xlims_on();




% figure




%% save 
filedir = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/';
save_fig=  true;
if save_fig
     m1 = 0.5;
     m2 = 1.2;
    filename = 'FRF_legs';
    width = 5.6;
    height = 7;
    fontsize = 9;
    setplot(h1,[width height]);
    % get the legend width
    lh = findobj(h1,'Type','axes','Tag','legend');
    set(lh,'units','centimeters');
    lp = get(lh,'position');
    leg_width = lp(3)+0.6;
    width = width+leg_width;  
    height = height*1.01;
    m3 = 0.1;
    m4 = 1;
    height
    setplot(h1,[width height],{m1,m2,m3,m4,leg_width,[]},fontsize);
    set(h1,'PaperUnits', 'centimeters');
    set(h1,'Papersize',[width height]);
    set(h1,'PaperPositionMode','manual');
    set(h1,'PaperPosition',[0 0 width height]);
    print(h1,'-dpdf',[filedir,filename])

    filename = 'FRF_trunk';
    width = 6.1;
    height = 7;
    height = height*1.01;
    m4 = 1+0.4;
    width = width+0.1;
    m3 = 0.1;
    height
    setplot(h2,[width height],{m1,m2,m3,m4,0.1,[]},fontsize);
    set(h2,'PaperUnits', 'centimeters');
    set(h2,'Papersize',[width height]);
    set(h2,'PaperPositionMode','manual');
    set(h2,'PaperPosition',[0 0 width height]);
    print(h2,'-dpdf',[filedir,filename])
    
end

