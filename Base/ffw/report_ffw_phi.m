clear all; close all; clc;
set(0,'defaulttextinterpreter','latex');


joint = 3;
joint_list = {'x','y','phi2'};

fig1 = figure; scr rt;
for i = [1,2,4]
    load(['Data/FFW_',joint_list{joint},'_0',num2str(i)]);
    n_plots =2; i_p = 1; 
    figure(fig1);
%     ax(joint,i_p) = subplot(n_plots,1,i_p); i_p = i_p+1;
%     plot(time,ref(:,joint)); ylabel('ref [m]'); grid on; hold all;
    % plot(time,enc(:,joint)); legend('ref','enc'); title(['joint ',num2str(joint)]);
%     ax(joint,i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
%     plot(time,uc(:,joint)); ylabel('control [Nm]'); grid on;  hold all;
%     plot(time,ffw(:,joint));
%     plot(time,u(:,joint)); legend('PD','ffw','C')
    
    ax(i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
    plot(time,cumsum(err(:,joint))./1000); ylabel('$e_\phi$ [m]'); hold all;
    ax(i_p) = subplot(n_plots,1,i_p);i_p = i_p+1;
    plot(time,err(:,joint)); ylabel('$\dot{e}_\phi$ [m/s]'); hold all;
    
    
end

% reference
acc = zeros(size(time));
accr = 1;
ts = 1; Ts=1000;
for j=1:3
    sine=1;
    if j==2, sine = -1; end
    acc(ts*1000+1:(ts+3)*1000) = ones(3000,1).*accr.*sine;
    acc((ts+6)*1000+1:(ts+9)*1000) = -ones(3000,1).*accr.*sine;
    ts = ts+11;
end
axes(ax(1)); 
title('Error in $\phi$ direction');
plot(time,acc*0.09,'k');
axes(ax(2));
plot(time,acc.*0.23,'k');

xlabel('Time [s]');
linkaxes(ax,'x');
set(ax(2),'xlim',[0 23]);
set(ax(2),'ylim',[-0.25, 0.25]);
set(ax(1),'ylim',[-0.1, 0.1]);
all_grids_on();
set(ax(1),'xticklabel',[]);

leg = legend_outside(fig1,{'No feed forward','$K_c$','$K_c$ and $K_a$','$\ddot{r}_\phi$ Reference'});
set(leg,'interpreter','latex');
%% save
save_fig = true; % set m1 = 0.1

filedir = '/home/ton/Dropbox/Linux/Report_final/Images/Base/';
if save_fig
    filename = 'base_ffw_phi';
    save_report(fig1,filedir,filename,'customleg',[10 5]);
end