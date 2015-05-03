clear all; close all; clc;

%% get measures from model
run torso_measures_NX

th_2_min = spindle2_to_angle2(min_spindle2);    % min angle q2
th_2_max = spindle2_to_angle2(max_spindle2);    % max angle q2
th_0_min = spindle1_to_angle0(min_spindle1);    % min angle q0
th_0_max = spindle1_to_angle0(max_spindle1);    % max angle q0


%% show max bend in upper position
q2_min = 70/180*pi;
fig3 = figure;
plot_torso_pos2(th_0_max,q2_min,true); hold all
plot_torso_pos2(th_0_max,th_2_min,true);
ylabel('Height [m]'); xlabel('Lenght [m]'); 
leg = legend_outside(fig3,{'limit','min'});
ax =gca;
xlim([0 0.5]);
yl= get(ax,'ylim');
xl = get(ax,'xlim');
grid on
set(leg,'units','centimeters');
leg_pos = get(leg,'position');
leg_width = leg_pos(3);
scale = 10;
m1 = 0.1; m2 = 1.0; m3 = 0; m4 = 1.2; m5 = leg_width+0.5; m6 = 0;
figsize = [xl(2) yl(2)].*scale+[m4+m5 m2+m1];
setplot(fig3,figsize,{m1,m2,m3,m4,m5,m6},11);

set(fig3,'PaperUnits', 'centimeters');
set(fig3,'Papersize',figsize);
set(fig3,'PaperPositionMode','manual');
set(fig3,'PaperPosition',[0 0 figsize]);

% dir_file = '/home/ton/Dropbox/Linux/Report/Images/Springs/trunk_limit_pos';
% print(fig3,'-dpdf',dir_file)