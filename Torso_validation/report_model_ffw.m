close all; clear all; clc;
plotsettings
set(0,'defaulttextinterpreter','latex');


load Data_model/stand_bend_PD_ffw.mat

qr = [spindle1_to_angle0(ref(:,1)),spindle2_to_angle2(ref(:,2))];
qm = [spindle1_to_angle0(enc(:,1)),spindle2_to_angle2(enc(:,2))];
qe = qr-qm;


%% plot results joint 1
% n_plots = 3; i_p = 1; joint =1;
% fig1 = figure; scr rt;
% ax(joint,i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
% % plot(time,ref(:,joint)); ylabel('ref [m]'); grid on; hold all;
% plot(time,qr(:,joint)); ylabel('$q_{0,r}$ [rad]'); grid on; hold all;
% % plot(time,enc(:,joint)); leg(1) = legend('ref','enc'); 
% title('Leg');
% ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
% % plot(time,err(:,joint)); ylabel('err [m]'); grid on;
% plot(time,qe(:,joint)); ylabel('$e_1$ [rad]'); grid on;
% ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,u(:,joint)); ylabel('$\tau_1$ [Nm]'); grid on;  hold all;
% plot(time,uc(:,joint));
% plot(time,ffw(:,joint)); leg = legend('$TOT$','$FB$','$FF$');
% xlabel('Time [s]');
% linkaxes(ax(joint,:),'x');
% set(leg,'interpreter','latex');
% %% plot results joint 2
% n_plots = 3; i_p = 1; joint =2;
% fig2 = figure; scr lt;
% ax(joint,i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
% % plot(time,ref(:,joint)); ylabel('ref [m]'); grid on; hold all;
% plot(time,qr(:,joint)); ylabel('$q_{2,r}$ [rad]'); grid on; hold all;
% % plot(time,enc(:,joint)); 
% title('Trunk');
% % leg2 =legend('ref','enc'); 
% ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
% % plot(time,err(:,joint)); ylabel('err [m]'); grid on;
% plot(time,qe(:,joint)); ylabel('$e_2$ [rad]'); grid on;
% ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,u(:,joint)); ylabel('$\tau_2$ [Nm]'); grid on;  hold all;
% plot(time,uc(:,joint));
% plot(time,ffw(:,joint)); leg2 = legend('$TOT$','$FB$','$FF$','location','northeastoutside');
% xlabel('Time [s]');
% linkaxes(ax(joint,:),'x');
% set(leg2,'interpreter','latex');
% 
% 
% set(ax,'xlim',[0 35]);
% set(ax(:,1:2),'xticklabel',[]);
%% plot results combined
n_plots = 4; i_p = 1; joint =2;
fig3 = figure; scr lt;
ax2(i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
% plot(time,ref(:,joint)); ylabel('ref [m]'); grid on; hold all;
plot(time,qr(:,joint)); ylabel('$q_r$ [rad]'); grid on; hold all;
plot(time,qr(:,1)); ylabel('$q_r$ [rad]');leg4 = legend('$q_2$','$q_0$','location','northeastoutside');
% plot(time,enc(:,joint)); 
% title('Trunk');
% leg2 =legend('ref','enc'); 
ax2(i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
% plot(time,err(:,joint)); ylabel('err [m]'); grid on;
plot(time,qe(:,joint)); ylabel('$e$ [rad]'); hold all;
plot(time,qe(:,1));
ax2(i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,u(:,joint)); ylabel('$\tau_2$ [Nm]'); grid on;  hold all;
plot(time,uc(:,joint));
plot(time,ffw(:,joint)); leg3 = legend('$TOT$','$FB$','$FF$','location','northeastoutside');
ax2(i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
plot(time,u(:,1)); ylabel('$\tau_0$ [Nm]'); grid on;  hold all;
plot(time,uc(:,1));
plot(time,ffw(:,1)); 
xlabel('Time [s]');
linkaxes(ax2,'x');
set(leg3,'interpreter','latex');
set(leg4,'interpreter','latex');

all_grids_on();
% all_xlims_on();
set(ax2,'xlim',[0 35]);
all_ylims_on();
set(ax2(2),'yticklabel',[-0.001,0,0.001])
set(ax2(1:3),'xticklabel',[]);

%% save
save_fig = false; % set m1 = 0.1

filedir = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/';
if save_fig
%     filename = 'trunk_mod_ffw_PD';
%     save_report(fig2,filedir,filename,'customleg',[12 6]);
%     filename = 'leg_mod_ffw_PD';
%     save_report(fig1,filedir,filename,'customleg',[12 6]);
    
    % set m1=0.1
%     filename = 'torso_mod_ffw_PD';
%     save_report(fig3,filedir,filename,'customleg',[12 8]);
end