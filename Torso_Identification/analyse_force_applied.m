clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analyse force angle and power %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

run torso_measures_NX
load /home/ton/Dropbox/Linux/Matlab/Torso/Model/motor_constants.mat


lAE = di(A,E);
lAC = di(A,C);
lJK = di(J,K);
lJH = di(J,H);
lJL = di(J,L);
g = 9.81;
M_T = 10;

% clear parameters
clearvars -except lAE lAC lJK lJH lJL g Kelm Kmm rsp r_gear2 M_T

pos1 = {'up','c','do'};
ref1 = [0.38, 0.325, 0.27];
pos2 = {'up','c','do'};
ref2 = [0.44, 0.405, 0.37];
joint = 2;

tstop = [115, 55, 55;55, 55, 55;55, 55, 55];

format_string = 'Equilibrium Voltage trunk, %2s, %2s, model = %5.4f, measurement = %5.4f';
tau_mod = zeros(3);
tau_meas = tau_mod;
figure
for j=1:1:3
    for i=1
        load(['data/sine_tr_',pos1{i},'_',pos2{j}]);
        r1 = ref(:,2);
        u1 = u(:,2);
        
        ref_0 = ref2(j);
        t_stop = tstop(i,j);

        % plot data
%         figure;
%         subplot(2,1,1); plot(time,ref);
%         subplot(2,1,2); plot(time,u);
        
        % Compute direct force applied on joint
%         th_force = cosinerule(r1,lAE,lAC,[]);   % angle of force applied on leg
        th_force = cosinerule(r1,lJK,lJH,[]);   % angle of force applied on trunk
        u_eff = sin(th_force).*u1;
%         figure; 
%         plot(time,u1,time,u_eff)
        
        % Compute force applied
        Fsp = u(:,2).*(Kelm*Kmm*r_gear2*rsp);
        Fsp_eff = Fsp.*sin(th_force);
        
        % Compute gravity force needed
        th_1 = spindle1_to_angle0(ref(:,1));
        th_g = th_1-angle0_to_angle1(th_1)+spindle2_to_angle2(ref(:,2));
        Mg_mod = sin(th_g).*(lJL/2*M_T*g);
        Fsp_mod = Mg_mod./lJK;
        
%         figure
        subplot(2,1,1); plot(time,Fsp_eff,time,Fsp_mod); ylabel('F'); hold all;
        subplot(2,1,2); plot(time,th_g.*(360/2/pi)); ylabel('angle gravity'); hold all;
        
        
    end
end
linkaxes(get(gcf,'children'),'x');
%%

    
    
all_grids_on();




