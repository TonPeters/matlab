clear all; close all; clc;
plotsettings
set(0,'defaulttextinterpreter','latex');
% filename = 'Trunk_acc08_leg10';
filedir = 'Data_acc/';

trunk = '10';
% acc = 0.017;
% vel = 0.034;
acc = 0.5e-4;
filenames = {'Trunk_new_acc0_leg10','Trunk_new_acc15_leg10'};

% settings
duration = 7;
ref = zeros(14*1000+1,1);
ref(1:1500) = ones(1500,1).*acc;
ref(7001:8500) = ones(1500,1).*-acc;
ref(3001:4500) = ones(1500,1).*-acc;
ref(10001:11500) = ones(1500,1).*acc;

fig1 = figure; scr rt;
% subplot(2,1,1);
t = linspace(0,14,14001).';
pl(1) = plot(t,ref,'color',ps.list5{2}); hold all;
for i=1:2
    filename = filenames{i};
    load([filedir,filename]);
    
    figure(fig1);
%     subplot(2,1,2);
    %pl(i+1) = plot(t,e(1000:15000),'color',ps.list5{i+2}); hold all;
%     pause
    pl(i+1) = plot(t,e(15000:29000),'color',ps.list5{i+2});
%     pause
    
end
xlabel('Time [s]');
ylabel('Error [m]');
title('Trunk');
ylim([-6e-5, 6e-5]);
set_xlim(gca,0);
all_grids_on();
leg = legend(pl,{'Ref.','$K_{a}=0$','$K_a=0.15$'},'location','northeastoutside');
set(leg,'interpreter','latex')
uistack(pl(1),'top');
%% save
save_fig = false;

filedir = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/';
if save_fig
    filename = 'trunk_acc_ffw';
    save_report(fig1,filedir,filename,'customleg',[7 5]);
end


