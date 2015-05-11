clear all; 
close all; 
clc;

%% get measures from model
plotsettings
run torso_measures_NX

th_2_min = spindle2_to_angle2(min_spindle2);    % min angle q2
th_2_max = spindle2_to_angle2(max_spindle2);    % max angle q2
% th_0_min = spindle1_to_angle0(min_spindle1);    % min angle q0
th_0_max = spindle1_to_angle0(max_spindle1);    % max angle q0
th_0_min= 0.05;
min_spindle1 = angle0_to_spindle1(th_0_min);

% motor parameters
K_m     = 29.2e-3;      % Nm/A,         Motor torque constant
K_elm   = 10;           % A/V_input,    Gain from input Voltage to Current     
r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
l_ls    = 0.002;        % m,           Lead of the spindle
r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation
I_m     = 79.2e-7;      % kg.m2,        Motor rotor inertia
l_s = 0.25;             % m,            Estimate ?????? lenght of the rotating screw
I_ls = 10e-6.*l_s;      % kg.m2,        Spindle screw inertia
tau_nom_max = 101e-3;   % Nm            Maximum nominal motor torque
rpm_nom = 7000;         % rad/s         Nominal motor speed
enc_counts = 500*4;     % counts/round  Used to calculate resolution

% nominal motor speed rad
omega_nom = rpm_nom/60*2*pi;

% Range of the spindle
range = max_spindle2-min_spindle2;
max_spindle2_bend = angle2_to_spindle2(shoulders_zero(th_0_max));
range_bend = max_spindle2_bend-min_spindle2;
%% calculate the desired gear ratio
% gear by force
F_bend = 1550;      % force to cover bending region
F_complete = 2400;  % force to cover complete range
gear_des_bend = F_bend/r_sp/tau_nom_max;
gear_des_complete = F_complete/r_sp/tau_nom_max;
speed_gear_des = omega_nom/gear_des_bend/r_sp*1000
% gear by speed
duration = 4;
omega_sp = range/(duration-1)*r_sp;
gear_nom_speed = omega_nom/omega_sp;
nom_speed = range/(duration-1)*1000;
nom_speed_bend = range_bend/(duration-1)*1000;

% print results
format_spec1 = 'Desired gear %4.0fN by force is %3.1f:1 and by nom speed is %3.1f:1, nom speed = %4.1f mm/s\n';
fprintf(format_spec1,F_bend,gear_des_bend,gear_nom_speed,nom_speed_bend)
fprintf(format_spec1,F_complete,gear_des_complete,gear_nom_speed,nom_speed)
fprintf('\n');

%% calculate the desired motor torque
tau_bend = F_bend/r_sp/r_gear2;
tau_complete = F_complete/r_sp/r_gear2;

% print results
format_spec2 = 'Desired motor torque by  %4.0fN force is %4.0f mNm\n';
fprintf(format_spec2,F_bend,tau_bend*1000);
fprintf(format_spec2,F_complete,tau_complete*1000);
fprintf('\n');

%% show different available gear ratios
format_spec = 'Gear %4.1f:1, duration %2.0f sec on range, nominal rpm %5.0f, maximum force %5.0f N, resolution %7.5f mm/count, speed %5.1f mm/s\n';

% gear 4.3:1 (currently used)
gear =                  r_gear2;
duration =              4;
force = tau_nom_max*gear*r_sp;
omega_m = range/(duration-1)*r_sp*gear;
rpm = omega_m/2/pi*60;
resolution = 2*pi/enc_counts/gear/r_sp*1000;
speed = omega_nom/gear/r_sp*1000;
fprintf(format_spec,gear,duration,rpm,force,resolution,speed);

% gear 6:1 
gear =                  6/1;
duration =              4;
force = tau_nom_max*gear*r_sp;
omega_m = range/(duration-1)*r_sp*gear;
rpm = omega_m/2/pi*60;
resolution = 2*pi/enc_counts/gear/r_sp*1000;
speed = omega_nom/gear/r_sp*1000;
fprintf(format_spec,gear,duration,rpm,force,resolution,speed);

% gear 12:1 
gear =                  12/1;
duration =              4;
force = tau_nom_max*gear*r_sp;
omega_m = range/(duration-1)*r_sp*gear;
rpm = omega_m/2/pi*60;
resolution = 2*pi/enc_counts/gear/r_sp*1000;
speed = omega_nom/gear/r_sp*1000;
fprintf(format_spec,gear,duration,rpm,force,resolution,speed);

% gear 12:1 5 sec
gear =                  12/1;
duration =              5;
force = tau_nom_max*gear*r_sp;
omega_m = range/(duration-1)*r_sp*gear;
rpm = omega_m/2/pi*60;
resolution = 2*pi/enc_counts/gear/r_sp*1000;
speed = omega_nom/gear/r_sp*1000;
fprintf(format_spec,gear,duration,rpm,force,resolution,speed);

%% plot choise in gear ratio
plotsettings

n = 30;
gear = linspace(4,12,n);
force = tau_nom_max*gear*r_sp;
% omega_m = rpm_nom*2*pi/60;
duration = range*gear*r_sp/omega_nom +1;
omega_m9000 = 9000*2*pi/60;
omega_m12000 = 12000*2*pi/60;
duration9000 = range*gear*r_sp/omega_m9000 +1;
speed = omega_nom./gear./r_sp*1000;
speed9000 = omega_m9000./gear./r_sp*1000;
speed12000 = omega_m12000./gear./r_sp*1000;


fig1 = figure; 
ax(1) = subplot(2,1,1);
pl(1) = plot(gear,force,'color',ps.list_div_XL{2,7}); hold all;
plot(gear,ones(size(gear)).*F_bend,'--','color',ps.list_div_XL{3,6});
plot(gear,ones(size(gear)).*F_complete,'--','color',ps.list_div_XL{3,8});
ylabel('Force max [N]');
ax(2) = subplot(2,1,2);
pl(2) = plot(gear,speed,'color',ps.list_div_XL{1,6}); hold all;
pl(3) = plot(gear,speed12000,'color',ps.list_div_XL{1,8});
pl(4) = plot(gear,ones(size(gear)).*nom_speed,'--','color',ps.list_div_XL{3,6});
pl(5) = plot(gear,ones(size(gear)).*nom_speed_bend,'--','color',ps.list_div_XL{3,8});
ylabel('Speed [mm/s]');
xlabel('Gear ratio');

linkaxes(ax,'x');
set(ax(1),'xticklabel',[]);


axes(ax(1));
% legend_outside(fig1,{'max force','speed, nominal','speed, maximal','range, center to bend','range, complete'},pl)

% indicate gear ratios
set(ax(1),'ylim',[1000 4000])
yl1 = get(ax(1),'ylim');
yl2 = get(ax(2),'ylim');
axes(ax(1));
plot(r_gear2,yl1(1),'*','color',ps.list_div_XL{4,8});
axes(ax(2));
plot(r_gear2,yl2(1),'*','color',ps.list_div_XL{4,8});

save_fig = false;
if save_fig
    scale =0.8;
    dir = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/';
    save_report(fig1,dir,'trunk_gear','custom',[7.5 6].*scale);
end

all_grids_on();