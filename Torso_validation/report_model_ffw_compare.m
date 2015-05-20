close all; clear all; clc;
plotsettings
set(0,'defaulttextinterpreter','latex');

list = {'PID04','PD_ffw','PID_ffw'};
ft = figure;
fl = figure;
for jj=1:1:3
    
    
load(['Data_model/stand_bend_',list{jj},'.mat']);

qr = [spindle1_to_angle0(ref(:,1)),spindle2_to_angle2(ref(:,2))];
qm = [spindle1_to_angle0(enc(:,1)),spindle2_to_angle2(enc(:,2))];
qe = qr-qm;


% plot results joint 1
n_plots = 2; i_p = 1; joint =1;
figure(fl); scr rt;
% ax(joint,i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
% plot(time,ref(:,joint)); ylabel('ref [m]'); grid on; hold all;
% plot(time,qr(:,joint)); ylabel('$q_{0,r}$ [rad]'); grid on;
% plot(time,enc(:,joint)); leg(1) = legend('ref','enc'); 
% title('Leg');
ax(joint,i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
plot(time,qe(:,joint)); ylabel('$e_1$ [rad]'); hold all;title('Leg');
ax(joint,i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
plot(time,u(:,joint)); ylabel('$\tau_1$ [Nm]'); grid on;  hold all;
xlabel('Time [s]');
linkaxes(ax(joint,:),'x');

%% plot results joint 2
n_plots = 2; i_p = 1; joint =2;
figure(ft); scr lt;
% ax(joint,i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
% plot(time,ref(:,joint)); ylabel('ref [m]'); grid on; hold all;
% plot(time,qr(:,joint)); ylabel('$q_{2,r}$ [rad]'); 
% plot(time,enc(:,joint)); 
% leg2 =legend('ref','enc'); 
ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
title('Trunk');
% plot(time,err(:,joint)); ylabel('err [m]'); grid on;
plot(time,qe(:,joint)); ylabel('$e_2$ [rad]'); hold all;
ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,u(:,joint)); ylabel('$\tau_2$ [Nm]'); hold all;
xlabel('Time [s]');


end
legl = legend_outside(fl,{'$FB$','$FB+FF-I$','$FB+FF$'});
set(legl,'interpreter','latex');
legt = legend_outside(ft,{'$FB$','$FB+FF-I$','$FB+FF$'});
set(legt,'interpreter','latex');

linkaxes(ax(joint,:),'x');
% set(leg2,'interpreter','latex');


set(ax,'xlim',[0 35]);
set(ax(:,1),'xticklabel',[]);
set(ax(1,1),'ylim',[-1.1e-3 1.1e-3]);
set(ax(1,2),'ylim',[0 0.13]);
set(ax(2,1),'ylim',[-3e-3 3e-3]);
set(ax(2,2),'ylim',[-0.05 0.08]);
% set(ax(:,1),'ylim','
% set(ax(1,1),'yticklabel',[-0.002, 0, 0.002]);
% set(ax(2,1),'yticklabel',[-0.002, 0, 0.002]);
% all_ylims_on();
all_grids_on();


%% save
% setplot(fl,[12 6]);
% setplot(ft,[12 6]);
% set(ax(1,1),'yticklabel',[-0.001, 0, 0.001]);
% set(ax(2,1),'yticklabel',[-0.002, 0, 0.002]);


save_fig = false; 

filedir = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/';
if save_fig
    filename = 'trunk_mod_ffw_compare';
    save_report(ft,filedir,filename,'customleg',[12 6]);
    filename = 'leg_mod_ffw_compare';
    save_report(fl,filedir,filename,'customleg',[12 6]);
end