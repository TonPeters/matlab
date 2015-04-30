clear all; 
close all; 
clc;
%-------------------------------------------------------------------------%
% Model: Trunk
%-------------------------------------------------------------------------%

%% get measures from model
run torso_measures_NX

% m1 = M_LRL+M_LFL;
% m2 = M_UL;
% m3 = 20;        % mass of the trunk
lAG = di(A,G);
lAE = di(A,E);
lAC = di(A,C);
lGJ = di(G,J);
lJK = di(J,K);
lJH = di(J,H);
lDJ = di(D,J);
lJL = di(J,L);
aBAZ = an(B,A,A+[0;-1;0]);
th_leg = 0.56;
th_leg2 = 0.2;
g = 9.81;

th_2_min = spindle2_to_angle2(min_spindle2);
th_2_max = spindle2_to_angle2(max_spindle2);
l1 = lJL;
lcm1 = l1*0.7;
lF = lJK;
th_r = -th_leg+angle0_to_angle1(th_leg);
th_r2 = -th_leg2+angle0_to_angle1(th_leg2);
m1 = 8;
% m2 = 0;
m1_lcm1 = 2.63;

% clear parameters
clearvars -except l1 lcm1 lF th_r m1 th_2_min th_2_max g m1_lcm1 th_r2
plotsettings
%% load data to fit
PC = 'ton/git_ton';
tau_grav_shift = -0.033;
% tau_grav_shift = 0.0;
leg_list = {'m0 c','m10 c','m20 c','m0 d','m10 d'};
leg_list2 = {}; leg_count = 1;
q2 = linspace(th_2_min,th_2_max,20).';
n_tot = 0;


Kmm = 29.2e-3;             % Nm/A,     torque constant
Kelm = 10;

params = [];
for i=1:1:5
%     if i==3
%         continue;
%     end
    leg_list2{leg_count} = leg_list{i}; leg_count= leg_count+1;
    
    % 0kg 0.56rad
    if i==1
        load(['/home/',PC,'/matlab/Torso_Identification/add_mass/upper_3mass.mat']);
        data_angle  = th_0k; % measured angle
        data_offset{i} = th_r; % angle offset
        data_input  = u_0k; % control input
        m2{i}          = 0;
        q_min       = 1.1;  % minimum angle
        q_max       = 2.2;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 32*1000+1;   % center switching velocity
        vel_stop    = 64*1000+1;   % end negative velocity
    elseif i==2
    % 10kg 0.56rad
        load(['/home/',PC,'/matlab/Torso_Identification/add_mass/upper_3mass.mat']);
        data_angle  = th_10k; % measured angle
        data_offset{i} = th_r; % angle offset
        data_input  = u_10k; % control input
        m2{i}          = 10;
        q_min       = 1.16;  % minimum angle
        q_max       = 2.31;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 28*1000+1;   % center switching velocity
        vel_stop    = 58*1000+1;   % end negative velocity
    elseif i==3
    % 20kg 0.56rad
        load(['/home/',PC,'/matlab/Torso_Identification/add_mass/upper_3mass.mat']);
        data_angle  = th_20k; % measured angle
        data_offset{i} = th_r; % angle offset
        data_input  = u_20k; % control input
        m2{i}          = 20;
        q_min       = 1.6;  % minimum angle
        q_max       = 2.3;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 25*1000+1;   % center switching velocity
        vel_stop    = 60*1000+1;   % end negative velocity
    elseif i==4
    % 0kg 0.2rad
        load(['/home/',PC,'/matlab/Torso_Identification/add_mass/upper_2mass_leg02.mat']);
        data_angle  = th_0k2; % measured angle
        data_offset{i} = th_r2; % angle offset
        data_input  = u_0k2; % control input
        m2{i}          = 0;
        q_min       = 1.1;  % minimum angle
        q_max       = 2.3;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 28*1000+1;   % center switching velocity
        vel_stop    = 53*1000+1;   % end negative velocity
    elseif i==5
    % 10kg 0.2rad
        load(['/home/',PC,'/matlab/Torso_Identification/add_mass/upper_2mass_leg02.mat']);
        data_angle  = th_10k2; % measured angle
        data_offset{i} = th_r2; % angle offset
        data_input  = u_10k2; % control input
        m2{i}          = 10;
        q_min       = 1.1;  % minimum angle
        q_max       = 2.28;  % maximum angle
        vel_start   = 0*1000+1;    % point before start positive velocity
        vel_switch  = 27*1000+1;   % center switching velocity
        vel_stop    = 53*1000+1;   % end negative velocity
    end
    
    % set data sample points
    n_s = 1000; % number of samples used for fitting
    q_m_s{i} = linspace(q_min+0.01,q_max-0.01,n_s).';


    % figure
    % subplot(2,1,1); plot(data_angle); grid on;
    % subplot(2,1,2); plot(data_angle./180*pi+data_offset,data_input); grid on;
    % pause


    %% use data
    q_meas{i} = data_angle./180*pi+data_offset{i}; 
%     qF0 = spindle2_to_Fangle2(angle2_to_spindle2(q_meas{i}));
    tau_meas{i} = data_input.*(Kelm*Kmm);

    for k=1:1:2
        if k==1;
            indices = (vel_start:vel_switch).';
            
        else
            indices = (vel_switch:vel_stop).';
            
        end
            
        % get data
        q_m = q_meas{i}(indices);
        tau_m = tau_meas{i}(indices);

        % cut data
        indi_p = find(q_m>q_min & q_m<q_max);
        q_m = q_m(indi_p);
        tau_m = tau_m(indi_p);

        %% filter data

        % lowpass filter using butterworth
        tau_m_filt = filter_data(tau_m,4,0.1);

        % remove datapoints that have the same encoder position and sample data
        [q_m_f,IA,IC] = unique(q_m);
        tau_s = interp1(q_m_f,tau_m_filt(IA),q_m_s{i});
        
        % save data
        sdata{i}.q2_m{k} = q_m;
        sdata{i}.tau_m{k} = tau_m;
%         sdata{i}.time_m{k} = time_m;
        sdata{i}.q2_s{k} = q_m_s{i};
        sdata{i}.tau_f{k} = tau_s;
        sdata{i}.qd2{k} = 0.004*(-1)^(k-1);
    end
    % save data
    sdata{i}.q_offset = data_offset{i};
    sdata{i}.m2 = m2{i};
    

%     % compute gravity term
%     tau_grav{i} = (tau_p+tau_n)./2+tau_grav_shift;
% 
%     % compute friction term
%     tau_fric_p{i} = tau_p-(tau_grav{i}-tau_grav_shift);
%     tau_fric_n{i} = tau_n-(tau_grav{i}-tau_grav_shift);    
% 
% 
%     figure(fig11);
%     plot(q_meas{i},tau_meas{i},'--','color',ps.list3{i}); hold all;
%     
%     figure(fig1);
%     ax1 = subplot(2,1,1);
%     plot(q_meas{i}(vel_start:vel_switch)-data_offset{i},tau_meas{i}(vel_start:vel_switch)+0.02*i,'--','color',ps.list3{i}); hold all;
%     ax2 = subplot(2,1,2);
%     plot(q_meas{i}(vel_switch:vel_stop)-data_offset{i},tau_meas{i}(vel_switch:vel_stop)+0.02*i,'--','color',ps.list3{i}); hold all;
    % plot((q_m-data_offset)./pi*180,tau_m,':','color',ps.list{i}); hold all;
%     plot((q_m_s{i}-data_offset{i})./pi*180,tau_grav{i},'color',ps.list{i}); hold all;
%     plot((q_m_s{i}-data_offset{i})./pi*180,tau_fric_p{i},'--','color',ps.list{i}); hold all;
%     plot((q_m_s{i}-data_offset{i})./pi*180,tau_fric_n{i},'-.','color',ps.list{i}); hold all;

%     n_tot = n_tot+length(q_m_s{i});
end
save('../Trunk_2/data_trunk','sdata');





all_grids_on();
