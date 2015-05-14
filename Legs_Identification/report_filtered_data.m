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
g = 9.81;

th_2_min = spindle2_to_angle2(min_spindle2);
th_2_max = spindle2_to_angle2(max_spindle2);
th_0_min = spindle1_to_angle0(min_spindle1);
th_0_max = spindle1_to_angle0(max_spindle1);
l3 = lJL;
% lcm3 = l1*0.7;
lF2 = lJK;
lF1 = lAE;
l1 = lAG;
l2 = lGJ;
l3 = lJL;

% estimates
lcm1 = l1/2;
lcm2 = l2/2;
lcm3 = l3/2;
m1 = 7;
m2 = 7;
% m3 = m3_lcm3/lcm3;
m3 = 7;

% m3_lcm3 = 2.7955;
% m3_lcm3 = 2.5;
% m3_lcm3 = 2.1947;
% m3_lcm3 = 2.6286;
m3_lcm3 = 2.4724;



% clear parameters
clearvars -except g m3_lcm3 l1 l2 l3 lcm1 lcm2 lcm3 m1 m2 m3 lF1 th_0_min th_0_max m3_lcm3
plotsettings
set(0,'defaulttextinterpreter','latex');
%% fit data
xsp0_max = angle0_to_spindle1(th_0_max);
xsp0_min = angle0_to_spindle1(th_0_min);
xsp0_n = linspace(xsp0_min,xsp0_max,20).';
q0 = spindle1_to_angle0(xsp0_n);
qd0 = ones(size(q0));
q0 = [q0];%;q0];
qd0 = [qd0];%;-qd0];
q1 = angle0_to_angle1(q0);

% Gear ratio joint 1 depending on joint 0
% r_q1_q0 = diff(angle0_to_angle1(q0));
% r_q1_q0 = [r_q1_q0;r_q1_q0(end)]./q0;
% r_q1_q0 =  1.8546;

%% iterate over data sets filter and sample data
joint = 1;
load_list = {'10','12','14','18','20','22'};
q2_list = [1.01, 1.4, 1.8,2.0, 2.2];
params = zeros(3,4);
n_s = 1000;
q0_m_s = linspace(th_0_min,th_0_max,n_s).';

Kmm = 29.2e-3;             % Nm/A,     torque constant
Kelm = 10;

% fig1 = figure; scr rb;
fig2 = figure; scr rt;
ax(1) = subplot(1,2,1);
ax(2) = subplot(1,2,2);

for i = 1:5
    %% load data
    load(['data_friction_new/m0kg_vel004_trunk',load_list{i},'.mat']);
    q0_meas = spindle1_to_angle0(enc(:,joint));
    qd0_meas = ones(size(q0_meas));
    qd0_meas(indices.n1) = -ones(length(indices.n1),1);
    qd0_meas(indices.n2) = -ones(length(indices.n2),1);
    tau_meas = u(:,1);
    
%     figure(fig2);
%     plot(q0_meas./pi*180,tau_meas.*Kelm*Kmm,'-','color',ps.list_div_XL{i,3}); hold all; 
    
    %% define sample range
    q0_min = max([min(q0_meas(indices.p1)),min(q0_meas(indices.p2)),min(q0_meas(indices.n1)),min(q0_meas(indices.n2))]);
    q0_max = min([max(q0_meas(indices.p1)),max(q0_meas(indices.p2)),max(q0_meas(indices.n1)),max(q0_meas(indices.n2))]);
    indi_q0 = find(q0_m_s>q0_min & q0_m_s<q0_max);
    q0_m_s_i = q0_m_s(indi_q0);

    %% data to fit
%     q0_m = [q0_meas(indices.p1);q0_meas(indices.p2);q0_meas(indices.n1);q0_meas(indices.n2)];
%     qd0_m = [qd0_meas(indices.p1);qd0_meas(indices.p2);qd0_meas(indices.n1);qd0_meas(indices.n2)];
%     tau_m = [tau_meas(indices.p1);tau_meas(indices.p2);tau_meas(indices.n1);tau_meas(indices.n2)];
    dir_vel = [1 1 -1 -1];
    indices_list = {indices.p1, indices.p2, indices.n1, indices.n2};
    indices_s = indices.p1;
    
    
    for k=[1 3] %length(indices_list)
        
        
        indices = indices_list{k};
        
%         indi = find(q0_meas(indices)<57/180*pi);
        q0_m = [q0_meas(indices)];
        qd0_m = [qd0_meas(indices)];
        tau_m = [tau_meas(indices)];
        time_m = time(indices);

        
        
        % filter data
        tau_m_filt = filter_data(tau_m,4,0.1);
        
        
        % sample data
        [q0_m_f,IA,IC] = unique(q0_m); % filter out same encoder positions
        tau_m_s{k} = interp1(q0_m_f,tau_m_filt(IA),q0_m_s_i);
        
        % plot data, filtered and sampled
        if (k==1 | k==2)
            axes(ax(1));
            plot(q0_meas(indices)./pi*180,tau_meas(indices).*Kelm*Kmm,'-','color',ps.list_div_XL{i,3}); hold all; 
            pl_s(i,k) = plot(q0_m_s_i./pi*180,tau_m_s{k}.*Kelm*Kmm,'color',ps.list_div_XL{i,7},'linewidth',ps.linewidth); hold all;
        else
            axes(ax(2));
            plot(q0_meas(indices)./pi*180,tau_meas(indices).*Kelm*Kmm,'-','color',ps.list_div_XL{i,3}); hold all; 
            pl_s(i,k) = plot(q0_m_s_i./pi*180,tau_m_s{k}.*Kelm*Kmm,'color',ps.list_div_XL{i,7},'linewidth',ps.linewidth); hold all;
        end
        
%         figure(fig2);
%         plot(q0_m./pi*180,tau_m_filt.*Kelm*Kmm,'-','color',ps.list_div_XL{i,6}); hold all;
%         pl_s(i,k) = plot(q0_m_s_i./pi*180,tau_m_s{k}.*Kelm*Kmm,'color',ps.list_div_XL{i,7},'linewidth',ps.linewidth); hold all;
        
        % save data
        sdata{i}.q0_m{k} = q0_m;
        sdata{i}.tau_m{k} = tau_m.*Kelm*Kmm;
        sdata{i}.time_m{k} = time_m;
        sdata{i}.q0_s{k} = q0_m_s_i;
        sdata{i}.tau_f{k} = tau_m_s{k}.*Kelm*Kmm;
        sdata{i}.qd0{k} = 0.004*dir_vel(k);
        
        err = find(isnan(tau_m_s{k}));
        if (~isempty(err))
            i
            k
            pause
        end
    end
    % save data
    sdata{i}.q2_list = q2_list(i);
    sdata{i}.q2 = spindle2_to_angle2(mean(enc(:,2)));
    
    
%     % take mean of 2 measurement sets
%     tau_p = (tau_m_s{2}+tau_m_s{1})./2;
%     tau_n = (tau_m_s{3}+tau_m_s{4})./2;
%     
%     % compute gravity term
%     tau_grav{i} = (tau_p+tau_n)./2;
%     
%     % compute friction term
%     tau_fric_p{i} = tau_p-tau_grav{i};
%     tau_fric_n{i} = tau_n-tau_grav{i};    
% 
%     figure(fig1)
%     plot(q0_m_s./pi*180,tau_grav{i},'color',ps.list{i+2}); hold all;
%     plot(q0_m_s./pi*180,tau_fric_p{i},'--','color',ps.list{i+2}); hold all;
%     plot(q0_m_s./pi*180,tau_fric_n{i},'-.','color',ps.list{i+2}); hold all;
    

end
leg_list = {};
for i=1:5
    for k=[1 3]
        uistack(pl_s(i,k),'top');
    end
    leg_list{i} = ['$q_2=',num2str(sdata{i}.q2,2),'$'];
end


%% save figure
figure(fig2);
all_grids_on();
all_xlims_on();
all_ylims_on();
linkaxes(ax,'x');
% set(ax(2),'yticklabel',[]);
% ylim([0 0.12]);
axes(ax(1));
xlabel('angle $q_0$ [degree]');
ylabel('$\tau_{motor}$ [Nm]');
title('Positive velocity');
axes(ax(2));
xlabel('angle $q_0$ [degree]');
title('Negative velocity');
legend(pl_s(:,3),leg_list,'location','southwest');


dir_file = '/home/ton/Dropbox/Linux/Report_final/Images/Torso/';
save_report(fig2,dir_file,'meas_legs','paperwidth');
