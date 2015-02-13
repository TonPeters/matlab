clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compare measured frf to model %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% load measured frfs
H1 = {};
hz1= {};
H2 = {};
hz2 = {};
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
                H1{count} = H;
                hz1{count} = hz;
            else
                H2{count} = H;
                hz2{count} = hz;
            end
        end
        count = count+1;
    end
end
clearvars -EXCEPT H1 hz1 H2 hz2
plotsettings
%% load model sys
load '/home/ton/Dropbox/Linux/Matlab/Torso/Model/sys_volt_spindle'
bod1 = figure; bod2 = figure;
w = logspace(-4,3,1000);
for i=1:1:9
    if i~=2 && i~=3
        Hm1 = squeeze(freqresp(sys{i}(1,1),w));
        figure(bod1); scr lt;
        subplot(2,1,1);
        semilogx(hz1{i},db(H1{i}),'b'); hold all;
        semilogx(w,db(Hm1),'color',ps.green);
        ylabel('Magnitude db'); set(gca,'xtick',[])
        subplot(2,1,2);
        semilogx(hz1{i},angle(H1{i}).*360/2/pi,'b'); hold all;
        semilogx(w,angle(Hm1).*360/2/pi,'color',ps.green);
        ylabel('angle degrees'); xlabel('frequency Hz');
    end
    
    
    Hm2 = squeeze(freqresp(sys{i}(2,2),w));
    figure(bod2); scr rt;
    subplot(2,1,1);
    semilogx(hz2{i},db(H2{i}),'b'); hold all;
    semilogx(w,db(Hm2),'color',ps.green);
    ylabel('Magnitude db'); set(gca,'xtick',[])
    subplot(2,1,2);
    semilogx(hz2{i},angle(H2{i}).*360/2/pi,'b'); hold all;
    semilogx(w,angle(Hm2).*360/2/pi,'color',ps.green);
    ylabel('angle degrees'); xlabel('frequency Hz');
    
    all_grids_on();
%     pause
%     close all;
end

%%
% save_report(bod1,'/home/ton/Dropbox/Linux/Report/Images/Model/','frf_mod_legs','custom',[10 6]);
% save_report(bod2,'/home/ton/Dropbox/Linux/Report/Images/Model/','frf_mod_trunk','custom',[10 6]);