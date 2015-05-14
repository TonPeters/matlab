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
% load '/home/ton/Dropbox/Linux/Matlab/Torso/Model/sys_volt_spindle'
h1 = figure; scr rt;
h2 = figure; scr lt;
leg_list1 = {};leg_count1 = 1;
leg_list2 = {};leg_count2 = 1;
H_sum1 = zeros(size(H1{1}));
H_sum2 = zeros(size(H2{1}));
for i=1:1:9
    % plot bode legs
    figure(h1);
    if i~=2 && i~=3
        subplot(2,1,1);
        semilogx(hz1{i},db(H1{i})); hold all; 
        ylabel('Magnitude db');
        subplot(2,1,2);
        semilogx(hz1{i},angle(H1{i}).*360/2/pi); hold all; 
        ylabel('Phase degrees'); xlabel('frequency Hz');
        leg_list1{leg_count1} = num2str(i);leg_count1 = leg_count1+1;
        legend(leg_list2);
        H_sum1 = H_sum1+H1{i};
    end
    
    % plot bode trunk
    figure(h2);
    subplot(2,1,1);
    semilogx(hz2{i},db(H2{i})); hold all;
    ylabel('Magnitude db');
    subplot(2,1,2);
    semilogx(hz2{i},angle(H2{i}).*360/2/pi); hold all;
    ylabel('Phase degrees'); xlabel('frequency Hz');
    leg_list2{leg_count2} =  num2str(i); leg_count2 = leg_count2+1;
    legend(leg_list1);
    H_sum2 = H_sum2+H2{i};
    
    all_grids_on();
end

Havg1 = H_sum1./7;
hzavg1 = hz1{1};
Havg2 = H_sum2./9;
hzavg2 = hz2{1};

