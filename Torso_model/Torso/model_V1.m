clear all; 
close all; 
clc;
%-------------------------------------------------------------------------%
% Model: Trunk
%-------------------------------------------------------------------------%

%% get measures from model
run torso_measures_NX

% temporary parameters
lAG = di(A,G);
lAE = di(A,E);
lAC = di(A,C);
lGJ = di(G,J);
lJK = di(J,K);
lJH = di(J,H);
lDJ = di(D,J);
lJL = di(J,L);
aBAZ = an(B,A,A+[0;-1;0]);

% parameters
g = 9.81;                                       % gravitation
th_2_min = spindle2_to_angle2(min_spindle2);    % min angle q2
th_2_max = spindle2_to_angle2(max_spindle2);    % max angle q2
th_0_min = spindle1_to_angle0(min_spindle1);    % min angle q0
th_0_max = spindle1_to_angle0(max_spindle1);    % max angle q0
l_F2 = lJK;                                      % arm of applied spindle force on joint 2
l_F1 = lAE;                                      % arm of applied spindle force on joint 1
l_1 = lAG;
l_2 = lGJ;
l_3 = lJL;

% estimates
l_cm1 = l_1/2;
l_cm2 = l_2/2;
l_cm3 = l_3/2;
m_1 = 7;
m_2 = 7;
m_3 = 7;



% clear parameters
clearvars -except g l_1 l_2 l_3 l_F1 l_F2 th_2_min th_2_max th_0_min th_0_max g
plotsettings

%% Settings
m4 = 20;     % mass of the arms
n = 8;     % grid size


%% system parameters
% known parameters
K_m     = 29.2e-3;      % Nm/A,         Motor torque constant
K_elm   = 10;           % A/V_input,    Gain from input Voltage to Current     
r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
l_ls    = 0.002;        % mm,           Lead of the spindle
r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation

% handy parameters
r_VtoF1 = (K_elm*K_m*r_gear1*r_sp); % N/V,  translation factor from voltage to spindle force 1
r_VtoF2 = (K_elm*K_m*r_gear2*r_sp); % N/V,  translation factor from voltage to spindle force 2

% estimated parameters
P1 = 6.0848;                    % P1 = lcm1*m1 + l1*m2 + l1*m3
P2 = 5.0254;                    % P2 = lcm2*m2 + l2*m3
P3 = 2.4724;                    % P4 = lcm3*m3
q2_gravity_offset = 2/180*pi;   % offset angle of gravity of joint q2
K_c1 = 0.0736.*r_VtoF1;         % N,    Coulomb friction force 1
K_c2 = 0.0995.*r_VtoF2;         % N,    Coulomb friction force 2
K_offset2 = -0.033.*r_VtoF2;    % N,    Direction dependant coulomb friction force 2




%% create variable set
% joint velocities
dq0 = 1;
dq2 = 1;
dq_ = [1; -1];

% joint angles
q0_ = linspace(th_0_min,th_0_max,n);
q2_ = linspace(th_2_min,th_2_max,n);
[q0,q2,dq] = meshgrid(q0_,q2_,dq_);

q1 = angle0_to_angle1(q0);



% other parameters depenant on joint angles
thF1 = spindle1_to_Fangle1(angle0_to_spindle1(q0)); % angle of force applied by spindle 1
thF2 = spindle2_to_Fangle2(angle2_to_spindle2(q2)); % angle of force applied by spindle 2
dq1_q0 = ratio_dq1_q0(q0);                          % change of q1 dependant on q0


%% Compute system model
% Joint Torque legs
T_leg = g.*(cos(q0).*(P1+l_1*m4)+...             
    (dq1_q0-1).*cos(q1-q0).*(P2+l_2*m4)+...     
    (1-dq1_q0).*cos(q2-q1+q0).*(P3+l_3*m4));

% Joint Torque arms
T_trunk = (g.*(P3+m4*l_3).*cos(q2-q1+q0+q2_gravity_offset));

% Spindle forces needed
F_leg = T_leg./l_F1./sin(thF1);  % 1 Gravity spindle force needed
F_fric1 = K_c1.*sign(dq);      % 1 Friction force needed
F_sp1 = F_leg + F_fric1;        % 1 Total spindle force needed
F_trunk = T_trunk./l_F2./sin(thF2);  % 1 Gravity spindle force needed
F_fric2 = K_c1.*sign(dq);      % 1 Friction force needed
F_sp2 = F_trunk + F_fric2;        % 1 Total spindle force needed

% Motor torque
M_m1 = F_sp1./(r_gear1*r_sp);
M_m2 = F_sp2./(r_gear2*r_sp);
% max(max(max(abs(M_m1))))
% max(max(max(abs(M_m2))))
% figure;
% subplot(2,1,1)
% surf(squeeze(q0(:,:,1))./pi*180,squeeze(q2(:,:,1))./pi*180,squeeze(M_m1(:,:,1)));
% subplot(2,1,2)
% surf(squeeze(q0(:,:,1))./pi*180,squeeze(q2(:,:,1))./pi*180,squeeze(M_m2(:,:,1)));

% Motor input Voltage
V_1 = F_sp1./r_VtoF1;       % Requiered input voltage motor 1
V_2 = F_sp2./r_VtoF2;       % Requiered input voltage motor 2

% Motor input Current
I_1 = F_sp1./(K_m*r_gear1*r_sp);       % Requiered input current motor 1
I_2 = F_sp2./(K_m*r_gear2*r_sp);       % Requiered input current motor 2

%% show results
figure; 
subplot(2,2,1);
plot(squeeze(q0(:,:,1)).'./pi*180,squeeze(I_1(:,:,1))); hold all; 
ylabel('Voltage (+ vel)');ax1 = gca;
subplot(2,2,3)
plot(squeeze(q0(:,:,2)).'./pi*180,squeeze(I_1(:,:,2)));
ylabel('Voltage (- vel)');ax3 = gca;
xlabel('angle joint 1 [deg]');
subplot(2,2,2);
plot(squeeze(q2(:,:,1))./pi*180,squeeze(I_2(:,:,1)));  ax2 = gca;
hold all;
subplot(2,2,4)
plot(squeeze(q2(:,:,1))./pi*180,squeeze(I_2(:,:,2))); ax4 = gca;
xlabel('angle joint 2 [deg]');
linkaxes([ax1 ax3],'xy')
linkaxes([ax2 ax4],'xy')

% define colormap
cmax1 = max(max(max(max(abs(I_1)))));
cmin1 = min(min(min(min(abs(I_1)))));
cmax2 = max(max(max(max(abs(I_2)))));
cmin2 = min(min(min(min(abs(I_2)))));
cm_length = 60;
CV_1 = fix((abs(I_1)-cmin1)./(cmax1-cmin1).*cm_length)+1;
CV_2 = fix((abs(I_2)-cmin2)./(cmax2-cmin2).*cm_length)+1;

figure; scr rt;
ax_1(1) = subplot(2,1,1);
surf(squeeze(q0(:,:,1))./pi*180,squeeze(q2(:,:,1))./pi*180,squeeze(I_1(:,:,1)),squeeze(CV_1(:,:,1)),...
    'CDataMapping','direct');
title('positive velocity');
xlabel('q_0 [deg]'); ylabel('q_2 [deg]'); zlabel('Current 1 (+vel)');
ax_1(2) = subplot(2,1,2);
surf(squeeze(q0(:,:,2))./pi*180,squeeze(q2(:,:,2))./pi*180,squeeze(I_1(:,:,2)),squeeze(CV_1(:,:,2)),...
    'CDataMapping','direct');
xlabel('q_0 [deg]'); ylabel('q_2 [deg]'); zlabel('Current 1 (-vel)');
col_leg = colorbar;
caxis([0 cm_length]);
xtick = get(col_leg,'Ytick');
Xlabel = {};
for i=1:1:length(xtick)
    Xlabel{i} = num2str(xtick(i)./cm_length.*(cmax1-cmin1)+cmin1,2);
end
set(col_leg,'YTickLabel',Xlabel)



figure; scr lt;
ax_2(1) = subplot(2,1,1);
surf(squeeze(q0(:,:,1))./pi*180,squeeze(q2(:,:,1))./pi*180,squeeze(I_2(:,:,1)),squeeze(CV_2(:,:,1)),...
    'CDataMapping','direct');
xlabel('q_0 [deg]'); ylabel('q_2 [deg]'); zlabel('Current 2 (+vel)');
ax_2(2) = subplot(2,1,2);
surf(squeeze(q0(:,:,2))./pi*180,squeeze(q2(:,:,2))./pi*180,squeeze(I_2(:,:,2)),squeeze(CV_2(:,:,2)),...
    'CDataMapping','direct');
xlabel('q_0 [deg]'); ylabel('q_2 [deg]'); zlabel('Current 2 (-vel)');
col_trunk = colorbar;
caxis([0 cm_length]);
xtick = get(col_trunk,'Ytick');
Xlabel = {};
for i=1:1:length(xtick)
    Xlabel{i} = num2str(xtick(i)./cm_length.*(cmax2-cmin2)+cmin2,2);
end
set(col_trunk,'YTickLabel',Xlabel)

hlink1 = linkprop(ax_1,{'CameraPosition','CameraUpVector'});
hlink2 = linkprop(ax_2,{'CameraPosition','CameraUpVector'});


%% invest current needed
I_1max = max(max(max((I_1))));
I_2max = max(max(max((I_2))));
I_1min = min(min(min((I_1))));
I_2min = min(min(min((I_2))));
range1 = I_1max-I_1min;
range2 = I_2max-I_2min;

format_str = 'Motor %1.0i, min current = %4.2f, max current = %4.2f, range = %4.2f \n';
fprintf(format_str,1,I_1min,I_1max,range1)
fprintf(format_str,2,I_2min,I_2max,range2)

%% motor characteristics
%   http://www.maxonmotor.nl/maxon/view/product/motor/dcmotor/re/re35/323890
%





























all_grids_on();
