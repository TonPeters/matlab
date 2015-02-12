clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fit gravity compensation %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

run torso_measures_NX
load /home/ton/Dropbox/Linux/Matlab/Torso/Model/motor_constants.mat


lAE = di(A,E);
lAC = di(A,C);
lJK = di(J,K);
lJH = di(J,H);
lJL = di(J,L);
g = 9.81;
M_T = 8;
lcm3 = lJL/1.5

% clear parameters
clearvars -except lAE lAC lJK lJH lJL g Kelm Kmm rsp r_gear2 M_T r_gear1 r_gear2 lcm3

pos1 = {'up','c','do'};
ref1 = [0.38, 0.325, 0.27];
pos2 = {'up','c','do'};
ref2 = [0.44, 0.405, 0.37];
joint = 2;
r_gear = r_gear1; if (joint==2), r_gear = r_gear2; end;

tstop = [115, 55, 55;55, 55, 55;55, 55, 55];

format_string = 'Equilibrium Voltage trunk, %2s, %2s, model = %5.4f, measurement = %5.4f';
Msp_mod = zeros(3);
Msp_meas = Msp_mod;
fig_g = figure;
for i=1:1:3
    for j=1:1:3
        load(['data/sine_tr_',pos1{i},'_',pos2{j}]);

        ref_0 = ref2(j);
        t_stop = tstop(i,j);

        % plot data
        figure;
        subplot(2,1,1); plot(time,ref);
        subplot(2,1,2); 

        % find start and end position
        tstart = find(ref(:,joint)<ref_0-0.001,1,'first');
        tend = find(ref(:,joint)>ref_0-0.001 & time<t_stop,1,'last');
        subplot(2,1,1); hold all; 
        plot(time(tstart),ref(tstart,joint),'r*',time(tend),ref(tend,joint),'r*');
        
        % Compute force applied
%         th_force = cosinerule(r1,lAE,lAC,[]);   % angle of force applied on leg
        th_force = cosinerule(ref(tstart:tend,joint),lJK,lJH,[]);   % angle of force applied on trunk
        Fsp = u(tstart:tend,joint).*(Kelm*Kmm*r_gear*rsp);
        Fsp_eff = Fsp.*sin(th_force);
        Msp_eff = Fsp_eff.*lJK;
        
        % compute gravity term
        Msp_meas(i,j) = mean(Msp_eff);
        subplot(2,1,2); 
        plot(time(tstart:tend),Msp_eff); hold all;
        plot(time(tstart:tend),ones(size(Msp_eff)).*Msp_meas(i,j));


        %% Equilibrium model
        % Compute gravity force needed
        th_1 = spindle1_to_angle0(ref(tstart:tend,1));
        th_g = th_1-angle0_to_angle1(th_1)+spindle2_to_angle2(ref(tstart:tend,2));
        Mg_mod = cos(th_g).*(lcm3*M_T*g);
%         Fsp_model = Mg_mod./lJK;
        Msp_mod(i,j) = mean(Mg_mod);
        plot(time(tstart:tend),Mg_mod,'r--');
        
        figure(fig_g);
        plot(time(tstart:tend),th_g.*180/pi); hold all;
        str = sprintf(format_string,pos1{i},pos2{j},Msp_mod(i,j),Msp_meas(i,j));
        disp(str);
        
    end
end

%%

figure;
for i=1:1:3
    th_1 = spindle1_to_angle0(ref1(i));
    th_g = th_1-angle0_to_angle1(th_1)+spindle2_to_angle2(ref2);
    plot(th_g.*180/pi,Msp_mod(i,:),'-*',th_g.*180/pi,Msp_meas(i,:),'--*'); hold all;
end
xlabel('angle joint 2 w.r.t. horizon [deg.]'); ylabel('Moment on joint [Nm]');
legend('mod up','meas up','mod c','meas c','mod do','meas do')
    
    
all_grids_on();




