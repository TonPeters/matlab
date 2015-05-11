clear all; close all; clc;

set(0,'defaulttextinterpreter','latex');
%% get measures from model
plotsettings
run torso_measures_NX

th_2_min = spindle2_to_angle2(min_spindle2);    % min angle q2
th_2_max = spindle2_to_angle2(max_spindle2);    % max angle q2
th_0_min = spindle1_to_angle0(min_spindle1);    % min angle q0
th_0_max = spindle1_to_angle0(max_spindle1);    % max angle q0

th_0_min = 0.05;    % min angle q0

ankle_z_height = 0.2465;
%% show max bend in upper position
fig1 = figure;

% get grid bounds
n = 10;
q2 = linspace(th_2_min,th_2_max,n);
q0 = linspace(th_0_min,th_0_max,n);
bounds_x = zeros(n,4);
bounds_y = zeros(n,4);
for i=1:1:n
    [bounds_x(i,1),bounds_y(i,1)] = plot_torso_end(q0(i),th_2_min,[1 1 1]);
    [bounds_x(i,2),bounds_y(i,2)] = plot_torso_end(q0(i),th_2_max,[1 1 1]);
    [bounds_x(i,3),bounds_y(i,3)] = plot_torso_end(th_0_min,q2(i),[1 1 1]);
    [bounds_x(i,4),bounds_y(i,4)] = plot_torso_end(th_0_max,q2(i),[1 1 1]);
end

% get zero bound
bounds_x_zero = [0 0];
bounds_y_zero = [0 0];
[bounds_x_zero(1), bounds_y_zero(1)] = plot_torso_end(th_0_min,shoulders_zero(th_0_min),[1 1 1]);
[bounds_x_zero(2), bounds_y_zero(2)] = plot_torso_end(th_0_max,shoulders_zero(th_0_max),[1 1 1]);

% plot grid bounds
pl(4) = plot(bounds_x(:,1),bounds_y(:,1),'--','color',ps.list_div_L{3,4});
plot(bounds_x(:,2),bounds_y(:,2),'--','color',ps.list_div_L{3,4});
plot(bounds_x(:,3),bounds_y(:,3),'--','color',ps.list_div_L{3,4});
plot(bounds_x(:,4),bounds_y(:,4),'--','color',ps.list_div_L{3,4});

% plot zero bounds
% pl(5) = plot(bounds_x_zero,bounds_y_zero,'--','color',ps.list_div_XL{4,7});

% plot AMIGO bounds
amigo_range = 0.32;
amigo_min = 0.725+0.15;
amigo_max = amigo_min+amigo_range;
amigo_x = [0 0];
amigo_y = [amigo_min amigo_max]-ankle_z_height;
pl(5) = plot(amigo_x,amigo_y,'-+','color',ps.list_div_XL{4,7});

n=2;
q2 = linspace(th_2_min,th_2_max,n);

% plot orientations
for i=1:1:n
    col=5;
    if i~=1 && i~=n, col = 2; end;
%     ax(1) = subplot(1,2,1);
    pl(1) = plot_torso_pos2(th_0_min,q2(i),ps.list_div_L{1,col});
%     ax(2) = subplot(1,2,2);
    pl(2) = plot_torso_pos2(th_0_max,q2(i),ps.list_div_L{2,col});
end

% decoration
pl(3) = plot_torso_base();
ylabel('Height $z$ [m]'); xlabel('Lenght $x$ [m]'); 

% axes(ax(2))
% plot_torso_base();
% xlabel('Lenght [m]'); 
% 
% linkaxes(ax(1:2),'xy')
xl = [-0.5 0.4];
yl = [0 1.2];
xlim(xl); 
ylim(yl);
% set(ax(2),'yticklabel',[]);
all_grids_on();

leg = legend(pl,{'Torso down','Torso up','Base size','Bounds reachable space','AMIGOs reachable space'},'location','northeastoutside');
set(leg,'units','centimeters','interpreter','latex');
leg_pos = get(leg,'position');
leg_width = leg_pos(3);

%% save figure
save_fig = false;
if save_fig
    % scale figure
    m1 = 0.3; m2 = 1.0; m3 = 0; m4 = 1.2; m5 = 0.3+leg_width; m6 = 0;
    scale = 4;
    plot_size = [diff(xl) diff(yl)].*scale;
    figsize = plot_size.*[1,1]+[m4+m5+m6 m1+m2+m3];
    setplot(fig1,figsize,{m1,m2,m3,m4,m5,m6},10);

    % show front of sergio
    set(gca,'units','normalized');
    ax_pos = get(gca,'position');

    annotation('arrow',ax_pos(1)+[0.02 0.1],ax_pos(2)+ax_pos(4)-[0.05 0.05])
    set(gca,'units','centimeters');


    set(fig1,'PaperUnits', 'centimeters');
    set(fig1,'Papersize',figsize);
    set(fig1,'PaperPositionMode','manual');
    set(fig1,'PaperPosition',[0 0 figsize]);

    dir_file = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/amigo_limits';
    print(fig1,'-dpdf',dir_file)
end