clear all; close all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Determine ratios between angles and spindles
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotsettings
set(0,'defaulttextinterpreter','latex');
run torso_measures_NX
ls_1_min = min_spindle1;
ls_1_max = max_spindle1;
ls_2_min = min_spindle2;
ls_2_max = max_spindle2;
% ls_1 = linspace(ls_1_min,ls_1_max).';
% ls_2 = linspace(ls_1_min,ls_1_max).';


th_0_min = spindle1_to_angle0(min_spindle1);
th_0_max = spindle1_to_angle0(max_spindle1);
th_2_min = spindle2_to_angle2(min_spindle2);
th_2_max = spindle2_to_angle2(max_spindle2);

% drive train dynamics
r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
l_ls    = 0.002;        % m,           Lead of the spindle
r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation

% qm1 = ls_1*r_sp*r_gear1;
% qm2 = ls_2*r_sp*r_gear2;
q0 = linspace(th_0_min,th_0_max).';
q2 = linspace(th_2_min,th_2_max).';
%% gear q0 to motor
syms q0_sym
r_sym = sym_partial_derivative(angle0_to_spindle1(q0_sym),q0_sym);
r_q0qm1 = double(subs(r_sym,q0_sym,q0))*r_sp*r_gear1;


fig1 = figure;
ax(2) = subplot(1,3,1);
plot(q0./pi*180,r_q0qm1);
xlabel('$q_0$ [degree]')
ylabel('Ratio');
title('$q_0:\theta_{m,1}$');

%% gear q1 to motor
syms q0_sym
r_sym = sym_partial_derivative(angle0_to_angle1(q0_sym),q0_sym);
r_q0q1 = double(subs(r_sym,q0_sym,q0));
r_q1qm1 = r_q0qm1./r_q0q1;

% figure;
% plot(q0./pi*180,r_q1qm1);
% xlabel('Joint angle $q_1$ [degree]')

figure(fig1);
ax(1) = subplot(1,3,2); 
plot(q0./pi*180,r_q1qm1);hold all; 
xlabel('$q_0$ [degree]');
title('$q_1:\theta_{m,1}$');
% ylabel('$q_1:q_0$');
% linkaxes(ax,'x');
% set(ax(1),'xticklabel',[]);

%% gear q2 to motor
syms q2_sym
r_sym = sym_partial_derivative(angle2_to_spindle2(q2_sym),q2_sym);
r_q2qm2 = double(subs(r_sym,q2_sym,q2))*r_sp*r_gear2;

figure(fig1);
ax(2) = subplot(1,3,3);
plot(q2./pi*180,r_q2qm2);
xlabel('$q_2$ [degree]')
title('$q_2:\theta_{m,2}$');
% ylabel('$\theta_{m,1}:q_1$');

all_xlims_on();
all_ylims_on();
all_grids_on();

%% save gear q0
save_fig=  true;
if save_fig
    filedir = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/';
    filename = 'Gear_ratios';
%     save_report(fig1,filedir,filename,'custom',[14 4]);
    width = 14;
    height = 4;
    fontsize = 9;
    
    setplot(fig1,[width height],{[],[],[],[],[],[]},fontsize);
    
    all_grids_on();
    
    set(fig1,'PaperUnits', 'centimeters');
    set(fig1,'Papersize',[width height]);
    set(fig1,'PaperPositionMode','manual');
    set(fig1,'PaperPosition',[0 0 width height]);

    print(fig1,'-dpdf',[filedir,filename])
end


