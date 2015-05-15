clear all; close all; clc;
plotsettings
set(0,'defaulttextinterpreter','latex');
% filename = 'Trunk_acc08_leg10';
filedir = 'Data_acc/';

trunk = '10';
% acc = 0.017;
% vel = 0.034;
acc = 1e-4;
filenames = {'Leg_acc0_trunk','Leg_acc15_trunk'};

% settings
duration = 7;
ref = zeros(14*1000+1,1);
ref(1:2000) = ones(2000,1).*acc;
ref(7001:9000) = ones(2000,1).*acc;
ref(4001:6000) = ones(2000,1).*-acc;
ref(11001:13000) = ones(2000,1).*-acc;

fig1 = figure; scr rt;
% subplot(2,1,1);
t = linspace(0,14,14001).';
pl(1) = plot(t,ref,'color',ps.list5{2}); hold all;
for i=1:2
    filename = filenames{i};
    load([filedir,filename,trunk]);
    
    figure(fig1);
%     subplot(2,1,2);
    pl(i+1) = plot(t,e(1000:15000),'color',ps.list5{i+2}); hold all;
    plot(t,e(15000:29000),'color',ps.list5{i+2});
    
end
xlabel('Time [s]');
ylabel('Error [m]');
title('Leg');
all_ylims_on();
all_grids_on();
leg = legend(pl,{'Ref.','$K_{a}=0$','$K_a=0.15$'},'location','northeastoutside');
uistack(pl(1),'top');
%% save
save_fig = true;

filedir = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/';
if save_fig
    filename = 'leg_acc_ffw';
    save_report(fig1,filedir,filename,'customleg',[7 5]);
end


