clear all; close all; 
% clc;
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
M_T = 12;
m3 = 8;
m2 = 6;
m1 = 6;
% m1 = M_LRL+M_LFL;
% m2 = M_UL;
% m3 = 20;        % mass of the trunk
l1 = di(A,G);
lcm1 = l1/4;
l2 = di(G,J);
lDJ = di(D,J);
lcm2 = l2-lDJ/4;
l3 = di(J,L);
lcm3 = l3/1.5;

% clear parameters
clearvars -except lAE lAC lJK lJH lJL g Kelm Kmm rsp r_gear2 r_gear1 r_gear2 m1 m2 m3 l1 lcm1 l2 lcm2 l3 lcm3 r_constr

pos1 = {'up','c','do'};
ref1 = [0.38, 0.325, 0.27];
pos2 = {'up','c','do'};
ref2 = [0.44, 0.405, 0.37];
joint = 1;
r_gear = r_gear1; if (joint==2), r_gear = r_gear2; end;

tsta = [0, 5, 0;0, 0, 0;0, 0, 0];
tstop = [55, 60, 55;55, 55, 55;55, 60, 55];

format_string = 'Equilibrium Voltage trunk, %2s, %2s, model = %5.4f, measurement = %5.4f';
Msp_mod = zeros(3);
Msp_meas = Msp_mod;
Msp_mod2 = Msp_mod;
for i=1:1:3
    for j=1:1:3
        if (i==3 && j<3)
            continue;
        end
        
        load(['data/sine_leg_',pos1{i},'_',pos2{j}]);

        ref_0 = ref1(i);
        t_stop = tstop(i,j);
        t_start = tsta(i,j);

        % plot data
        figure;
        subplot(2,1,1); plot(time,ref); title([pos1{i},' ',pos2{j}]);
        subplot(2,1,2); 

        % find start and end position
        tstart = find(ref(:,joint)<ref_0-0.001 & time>t_start,1,'first');
        tend = find(ref(:,joint)>ref_0-0.001 & time<t_stop,1,'last');
        subplot(2,1,1); hold all; 
        plot(time(tstart),ref(tstart,joint),'r*',time(tend),ref(tend,joint),'r*');
        
        % Compute force applied
        th_force = cosinerule(ref(tstart:tend,joint),lAE,lAC,[]);   % angle of force applied on leg
%         th_force = cosinerule(ref(tstart:tend,joint),lJK,lJH,[]);   % angle of force applied on trunk
        Fsp = u(tstart:tend,joint).*(Kelm*Kmm*r_gear*rsp);
        Fsp_eff = Fsp.*sin(th_force);
        Msp_eff = Fsp_eff.*lAE;
        
        Msp_meas(i,j) = mean(Msp_eff);
        subplot(2,1,2); 
        plot(time(tstart:tend),Msp_eff); hold all;
        plot(time(tstart:tend),ones(size(Msp_eff)).*Msp_meas(i,j));
        
        %% compute gravity term
        load /home/ton/Dropbox/Linux/Matlab/Torso/Model/motion_force_angle
    %     load /home/ton/Dropbox/Linux/Matlab/Torso/Model/motor_constants
        
        xsp1 = ref1(i); xsp2 = ref2(j);
        q_e = [spindle1_to_angle0(xsp1); spindle2_to_angle2(xsp2)];
    %     xsp2 = angle2_to_spindle2(q_e(2));
        qn = q_e;
        qdn = [0;0];
        S_n = double(subs(S,[q;qd],[qn;qdn]));
        H_n = double(subs(H,[q;qd],[qn;qdn]));
        tau_eq = inv(S_n)*H_n;
        tau_mod = tau_eq(joint);
        Fsp2 = tau_mod;
        th_force2 = cosinerule(xsp1,lAE,lAC,[]);   % angle of force applied on leg
    %     th_force2 = cosinerule(xsp2,lJK,lJH,[]);   % angle of force applied on trunk
        Fsp_eff2 = Fsp2.*sin(th_force2);
        Msp_mod(i,j) = Fsp_eff2.*lAE;
        
        %% Compute gravity term
        q0 = spindle1_to_angle0(ref1(i)); q1 = angle0_to_angle1(q0);
        q2 = spindle2_to_angle2(ref2(j)); 
        th0 = q0; th1 = q1-th0; th2 = q2-th1;
        % trunk
        F2 = m3*g;
        M2 = F2*lcm3*cos(th2);
        % upper leg
        F1 = F2+m2*g;
        M1 = -M2+F2*l2*cos(th1)+m2*g*lcm2*cos(th1);
        % lower leg
        F0 = F1+m1*g;
        M0 = -M1+F1*l1*cos(th0)+m1*g*lcm1*cos(th0);
        
        M_actuator = M0+M1.*r_constr;
        
        Msp_mod(i,j) = M_actuator;
        plot(time(tstart:tend),ones(size(Msp_eff)).*Msp_mod(i,j),'r--');
        
        
        str = sprintf(format_string,pos1{i},pos2{j},Msp_mod(i,j),Msp_meas(i,j));
        disp(str);
        
    end
end

%%

figure;
ref1 = spindle1_to_angle0(ref1);
for i=1:1:3
    if i==3
        plot(ref1,Msp_mod(:,i),'-*',ref1,Msp_meas(:,i),'--*'); hold all;
    else
        plot(ref1(1:2),Msp_mod(1:2,i),'-*',ref1(1:2),Msp_meas(1:2,i),'--*'); hold all;
    end
    
%     th_1 = spindle1_to_angle0(ref1(i));
%     th_g = th_1-angle0_to_angle1(th_1)+spindle2_to_angle2(ref2);
%     plot(th_g.*180/pi,Msp_mod(i,:),'-*',th_g.*180/pi,Msp_meas(i,:),'--*'); hold all;
end
xlabel('angle joint 2 w.r.t. horizon [deg.]'); ylabel('Moment on joint [Nm]');
legend('mod up','meas up','mod c','meas c','mod do','meas do')
    
    
all_grids_on();




