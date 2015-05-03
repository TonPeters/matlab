clear all; close all; clc;

%% get measures from model
plotsettings
run torso_measures_NX

th_2_min = spindle2_to_angle2(min_spindle2);    % min angle q2
th_2_max = spindle2_to_angle2(max_spindle2);    % max angle q2
th_0_min = spindle1_to_angle0(min_spindle1);    % min angle q0
th_0_max = spindle1_to_angle0(max_spindle1);    % max angle q0

th_0_min = 0.05;    % min angle q0

%% show max bend in upper position
fig1 = figure;

n=6;
q2 = linspace(th_2_min,th_2_max,n);

% plot orientations
for i=1:1:n
    col=5;
    if i~=1 && i~=n, col = 2; end;
    ax(1) = subplot(1,2,1);
    pl(1) = plot_torso_pos2(th_0_min,q2(i),ps.list_div_L{1,col});
    ax(2) = subplot(1,2,2);
    pl(3) = plot_torso_pos2(th_0_max,q2(i),ps.list_div_L{1,col});
end


% decoration
axes(ax(1))
plot_torso_base();
ylabel('Height [m]'); xlabel('Lenght [m]'); 

axes(ax(2))
plot_torso_base();
xlabel('Lenght [m]'); 

linkaxes(ax(1:2),'xy')
xl = [-0.5 0.4];
yl = [0 1.2];
xlim(xl); 
ylim(yl);
set(ax(2),'yticklabel',[]);
all_grids_on();

% scale figure
m1 = 0.3; m2 = 1.0; m3 = 0; m4 = 1.2; m5 = 0.3; m6 = 0.3;
scale = 5;
plot_size = [diff(xl) diff(yl)].*scale;
figsize = plot_size.*[2,1]+[m4+m5+m6 m1+m2+m3];
setplot(fig1,figsize,{m1,m2,m3,m4,m5,m6},10);

% 
% 
% ax =gca;
% ylim([0 1.2]);
% xlim([-0.5 0.4]);
% yl= get(ax,'ylim');
% xl = get(ax,'xlim');
% grid on
% leg_pos = [0,0,0,0];
% % set(leg,'units','centimeters');
% % leg_pos = get(leg,'position');
% leg_width = leg_pos(3);
% 
% 
% 
% 
% 
% set(fig1,'PaperUnits', 'centimeters');
% set(fig1,'Papersize',figsize);
% set(fig1,'PaperPositionMode','manual');
% set(fig1,'PaperPosition',[0 0 figsize]);

% dir_file = '/home/ton/Dropbox/Linux/Report/Images/Springs/trunk_limit_pos';
% print(fig3,'-dpdf',dir_file)