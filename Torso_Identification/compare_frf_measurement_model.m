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

%% load model sys
load '/home/ton/Dropbox/Linux/Matlab/Torso/Model/sys_volt_spindle'

w = logspace(-4,3,1000);
for i=1:1:9
    if i~=2 && i~=3
        Hm1 = squeeze(freqresp(sys{i}(1,1),w));
        figure; scr lt;
        subplot(2,1,1);
        semilogx(hz1{i},db(H1{i})); hold all;
        semilogx(w,db(Hm1));ylabel('Magnitude db');
        subplot(2,1,2);
        semilogx(hz1{i},angle(H1{i}).*360/2/pi); hold all;
        semilogx(w,angle(Hm1).*360/2/pi);ylabel('angle degrees'); xlabel('frequency Hz');
    end
    
    
    Hm2 = squeeze(freqresp(sys{i}(2,2),w));
    figure; scr rt;
    subplot(2,1,1);
    semilogx(hz2{i},db(H2{i})); hold all;
    semilogx(w,db(Hm2));ylabel('Magnitude db');
    subplot(2,1,2);
    semilogx(hz2{i},angle(H2{i}).*360/2/pi); hold all;
    semilogx(w,angle(Hm2).*360/2/pi);ylabel('angle degrees'); xlabel('frequency Hz');
    
    all_grids_on();
    pause
    close all;
end

