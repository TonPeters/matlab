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

m3_lcm3 = 2.7955;
m3_lcm3 = 2.5;


% clear parameters
clearvars -except g m3_lcm3 l1 l2 l3 lcm1 lcm2 lcm3 m1 m2 m3 lF1 th_0_min th_0_max m3_lcm3
plotsettings

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
r_q1_q0 =  1.8546;

%% iterate over data sets filter and sample data
joint = 1;
load_list = {'10','14','16'};
q2_list = [1.01, 1.4, 1.6];
Fc_list = [228.3295;  252.4930;  263.6036];
params = zeros(3,4);
n_s = 1000;
q0_m_s = linspace(18.6/180*pi,53.3/180*pi,n_s);

fig1 = figure; scr rb;
fig2 = figure; scr rt;
for i = 1:3
    %% load data
    load(['../../Legs_Identification/data_friction/m0kg_vel004_trunk',load_list{i},'.mat']);
    q0_meas = spindle1_to_angle0(enc(:,joint));
    qd0_meas = ones(size(q0_meas));
    qd0_meas(indices.n1) = -ones(length(indices.n1),1);
    qd0_meas(indices.n2) = -ones(length(indices.n2),1);
    tau_meas = u(:,1);
    
    figure(fig2);
    plot(q0_meas./pi*180,tau_meas,':','color',ps.list{i+2}); hold all; 
    
    %% data to fit
%     q0_m = [q0_meas(indices.p1);q0_meas(indices.p2);q0_meas(indices.n1);q0_meas(indices.n2)];
%     qd0_m = [qd0_meas(indices.p1);qd0_meas(indices.p2);qd0_meas(indices.n1);qd0_meas(indices.n2)];
%     tau_m = [tau_meas(indices.p1);tau_meas(indices.p2);tau_meas(indices.n1);tau_meas(indices.n2)];
    indices_list = {indices.p1, indices.p2, indices.n1, indices.n2};
    indices_s = indices.p1;
    for k=1:length(indices_list)
        indices = indices_list{k};
        
%         indi = find(q0_meas(indices)<57/180*pi);
        q0_m = [q0_meas(indices)];
        qd0_m = [qd0_meas(indices)];
        tau_m = [tau_meas(indices)];
        time_m = time(indices);


        %% filter data

        tau_m_filt = filter_data(tau_m,4,0.1);
        
        % sample data
        [q0_m_f,IA,IC] = unique(q0_m); % filter out same encoder positions
        tau_m_s{k} = interp1(q0_m_f,tau_m_filt(IA),q0_m_s);
        
        % plot data, filtered and sampled
        figure(fig2);
        plot(q0_m./pi*180,tau_m_filt,'-','color',ps.list{i+2}); hold all;
        plot(q0_m_s./pi*180,tau_m_s{k},'--','color',ps.list{i+2},'linewidth',ps.linewidth); hold all;
        
    end
    
    % take mean of 2 measurement sets
    tau_p = (tau_m_s{2}+tau_m_s{1})./2;
    tau_n = (tau_m_s{3}+tau_m_s{4})./2;
    
    % compute gravity term
    tau_grav{i} = (tau_p+tau_n)./2;
    
    % compute friction term
    tau_fric_p{i} = tau_p-tau_grav{i};
    tau_fric_n{i} = tau_n-tau_grav{i};    

    figure(fig1)
    plot(q0_m_s./pi*180,tau_grav{i},'color',ps.list{i+2}); hold all;
    plot(q0_m_s./pi*180,tau_fric_p{i},'--','color',ps.list{i+2}); hold all;
    plot(q0_m_s./pi*180,tau_fric_n{i},'-.','color',ps.list{i+2}); hold all;
    

end
figure(fig2)
ylim([0 0.4]);

%% average over data sets and substract friction component
% compute average friction and gravity
tau_grav_mean = (tau_grav{1}+tau_grav{2}+tau_grav{3})./3;
tau_fric_p_mean =  (tau_fric_p{1}+tau_fric_p{2}+tau_fric_p{3})./3;
tau_fric_n_mean =  (tau_fric_n{1}+tau_fric_n{2}+tau_fric_n{3})./3;
figure(fig1)
plot(q0_m_s./pi*180,tau_grav_mean,'color',ps.list{1},'linewidth',ps.linewidth); hold all;
plot(q0_m_s./pi*180,mean(tau_fric_p_mean).*ones(n_s,1),'color',ps.list{1},'linewidth',ps.linewidth); hold all;
plot(q0_m_s./pi*180,mean(tau_fric_n_mean).*ones(n_s,1),'color',ps.list{1},'linewidth',ps.linewidth); hold all;
% thF1 = spindle1_to_Fangle1(angle0_to_spindle1(q0_m_s));
% plot(q0_m_s./pi*180,tau_grav_mean.*sin(thF1),'color',ps.list{2},'linewidth',ps.linewidth); hold all;
% plot(q0_m_s./pi*180,mean(tau_fric_p_mean).*ones(1,n_s).*sin(thF1),'color',ps.list{2},'linewidth',ps.linewidth); hold all;
% plot(q0_m_s./pi*180,mean(tau_fric_n_mean).*ones(1,n_s).*sin(thF1),'color',ps.list{2},'linewidth',ps.linewidth); hold all;

legend('grav','coulomb +','coulom -')
mean(tau_fric_p_mean)
mean(tau_fric_n_mean)
mean([tau_fric_p_mean,-tau_fric_n_mean])


%% fit model to data


% figure(figc); ylim([0 0.4]);
%% save
% legend('meas 0kg','meas 10kg','meas 20kg','mod 0kg','mod 10kg','mod 20kg','Location','Eastoutside')
% save_report(gcf,'../../../../Report/Images/Model','trunk_gravity','customleg',[10 6]);

all_grids_on();
