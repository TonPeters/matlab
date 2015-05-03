clear all; close all; clc;

%% get measures from model
plotsettings
run torso_measures_NX

th_2_min = spindle2_to_angle2(min_spindle2);    % min angle q2
th_2_max = spindle2_to_angle2(max_spindle2);    % max angle q2
th_0_min = spindle1_to_angle0(min_spindle1);    % min angle q0
th_0_max = spindle1_to_angle0(max_spindle1);    % max angle q0

th_0_min = 0.05;    % min angle q0

% motor specs
r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
r_gear2 = 6/1;         % rad/rad,      Gear ratio from spindle to motor 2
l_ls    = 0.002;        % m,           Lead of the spindle
r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation
tau_nom_max = 101e-3;   % Nm            Maximum nominal motor torque
%% calculate forces
% plot grid
n = 10;
q0 = linspace(th_0_min,th_0_max,n);
q2 = linspace(th_2_min,th_2_max,n);
[q0_,q2_] = meshgrid(q0,q2);

% sim settings
N_trunk = 2;
F1_trunk = 675;
m_4 = 20;
qdd = 0.41;

F_lim = tau_nom_max*r_gear2*r_sp;

% simulate
F_trunk_max = zeros(size(q0_));
F_trunk_min = F_trunk_max;
% smart simulate
limit = false;
i0 = n;
i2 = 1;
while (~limit)
    F_trunk_max(i2,i0) = sim_trunk2_test_gear6(q0(i0),q2(i2),1,qdd,m_4,52,N_trunk,F1_trunk);
    if F_trunk_max(i2,i0)<F_lim
        if i2==1
            limit = true;
        else
            i0 = i0-1;
            i2 = 1;
        end
    else
        i2 = i2+1;
        if i2>n
            i2 = 1;
            i0 = i0-1;
        end
    end
end

limit = false;
i0 = 1;
i2 = n;
while (~limit)
    F_trunk_min(i2,i0) = sim_trunk2_test_gear6(q0(i0),q2(i2),-1,qdd,m_4,0,N_trunk,F1_trunk);
    if F_trunk_min(i2,i0)>-F_lim
        if i2==n
            limit = true;
        else
            i0 = i0+1;
            i2 = n;
        end
    else
        i2 = i2-1;
        if i2<1
            i2 = n;
            i0 = i0+1;
        end
    end
end

% F_trunk_min = sim_trunk2(q0_,q2_,-1,qdd,m_4,0,N_trunk,F1_trunk);



%% show max bend in upper position
fig1 = figure;


x = zeros(n,n);
y=x;
p_grid = x;
col = 2;
for i0=1:n
    for i2=1:n
        if F_trunk_max(i2,i0)<F_lim && F_trunk_min(i2,i0)>-F_lim
            [x(i0,i2),y(i0,i2),p_grid(i0,i2)] = plot_torso_end(q0(i0),q2(i2),ps.green);
        else
            [x(i0,i2),y(i0,i2),p_grid(i0,i2)] = plot_torso_end(q0(i0),q2(i2),ps.red);
        end
    end
end
% define legend
leg_g = plot(0,-1,'*','color',ps.green);
leg_r = plot(0,-1,'*','color',ps.red);


% plot grid bounds
plot(x(:,1),y(:,1),'--','color',ps.list_div_L{2,3});
plot(x(:,end),y(:,end),'--','color',ps.list_div_L{2,3});
plot(x(1,:),y(1,:),'--','color',ps.list_div_L{2,3});
plot(x(end,:),y(end,:),'--','color',ps.list_div_L{2,3});


% plot orientations
pl(1) = plot_torso_pos2(th_0_min,th_2_min,ps.list_div_XL{1,5},true);
pl(2) = plot_torso_pos2(th_0_min,th_2_max,ps.list_div_XL{1,5},true);
pl(3) = plot_torso_pos2(th_0_max,th_2_min,ps.list_div_XL{1,5},true);
pl(4) = plot_torso_pos2(th_0_max,th_2_max,ps.list_div_XL{1,5},true);
p_base = plot_torso_base();

for i0=1:n
    for i2=1:n
        uistack(p_grid(i0,i2),'top')
    end
end

% decoration
ylabel('Height [m]'); xlabel('Lenght [m]'); 
leg = legend([leg_g,leg_r,pl(1),p_base],{'Reachable','Unreachable','Torso orientation','Base size'},'location','northeastoutside');
set(leg,'units','centimeters','interpreter','latex');
leg_pos = get(leg,'position');
leg_width = leg_pos(3);

% set axes
ax =gca;
ylim([0 1.2]);
xlim([-0.5 0.4]);
yl= get(ax,'ylim');
xl = get(ax,'xlim');
grid on

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

%% save figure
set(fig1,'PaperUnits', 'centimeters');
set(fig1,'Papersize',figsize);
set(fig1,'PaperPositionMode','manual');
set(fig1,'PaperPosition',[0 0 figsize]);

dir_file = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/trunk_limit_new_gear';
print(fig1,'-dpdf',dir_file)