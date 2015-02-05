clear all; 
close all; 
clc;
%-------------------------------------------------------------------------%
% Model:
%   - rear leg as a point mass
%   - actuated at l_sp with angle dependancy
%   - upper leg as a point mass
%   - angle theta2 used with constraint equation
%   - lower gas spring as a linear spring (No direction dependancy)
%   - torso with spring
%   - arms as a point mass in the shoulders
%-------------------------------------------------------------------------%

%% get measures from model
run torso_measures_NX.m

m1 = M_LRL+M_LFL;
m2 = M_UL;
m3 = 20;        % mass of the trunk
lAG = di(A,G);
lAE = di(A,E);
lAC = di(A,C);
lGJ = di(G,J);
lJK = di(J,K);
lJH = di(J,H);
lDJ = di(D,J);
lJL = di(J,L);
aBAZ = an(B,A,A+[0;-1;0]);

% settings
N_leg = 2;      % number of springs in the legs (0 or 2)
N_trunk = 2;    % number of springs in the trunk (0 or 2)
m4 = 20;        % mass of the arms;
Bdamp = 0;

N_leg =0;
N_trunk =0;
m4 = 0;

th_0_min = spindle1_to_angle0(min_spindle1);
th_0_max = spindle1_to_angle0(max_spindle1);
th_2_min = spindle2_to_angle2(min_spindle2);
th_2_max = spindle2_to_angle2(max_spindle2);

% clear parameters
clearvars -except m1 m2 m3 m4 lAG lAE lAC lGJ lJK lJH lDJ lJL aBAZ K_leg L0_leg N_leg K_trunk L0_trunk N_trunk Bdamp th_0_min th_0_max th_2_min th_2_max

%% motor parameters
Kelm = 10;                  % Volt to current by elmo
R_m = 0.583;                % ohm
L_m = 0.191e-3;             % H
Km_m = 29.2e-3;             % Nm/A,     torque constant
Ke_m = 1/(328*2*pi/60);     % V/(rad/s) Back Emf constant (1/speed constant)
J_m = 79.2/1000/10000;      % kg*m^2
B_m = 1/(6.55*2*pi/60*1000);% Nm/(rad/s) motor friction
l_ls = 0.002;               % m,        lead of the led screw
d_ls = 0.012;               % m,        diameter led screw
r_ls = 2*pi/l_ls;            % gear th_ls/x_ls
r_gear1 = 5/2;              % spindle/motor
r_gear2 = 13/3;             % spindle/motor


%% other parameters

g=9.81;
s = tf('s');

% define outputs
C = [1 0 0 0;0 1 0 0];
D = [0 0; 0 0];

Bopt = bodeoptions;
Bopt.FreqUnits = 'Hz';

%% set variables
syms th0 th0d th0dd th1 th1d th1dd th2 th2d th2dd Fsp1 Fsp2
% coordinates (not independant)
q = [th0;th1;th2]; 
qd = [th0d;th1d;th2d];
qdd = [th0dd;th1dd;th2dd];
n_q = length(q);
tau = [Fsp1;Fsp2];
n_t = length(tau);

% minimal coordinates (independant)
q_min = [q(1);q(3)];
qd_min = [qd(1);qd(3)];
qdd_min = [qdd(1);qd(3)];
n_min = length(q_min);

% q writen in minimal coordinates
q_qmin = [q_min(1);angle0_to_angle1(q_min(1));q_min(2)];

% set equilibrium points (center points)
th_2_center = (th_2_min+th_2_max)/2; 
th_0_center = (th_0_min+th_0_max)/2;
q_e_list = [    th_0_min,       th_2_min;...
                th_0_center,    th_2_min;...
                th_0_max,       th_2_min;...
                th_0_min,       th_2_center;...
                th_0_center,    th_2_center;...
                th_0_max,       th_2_center;...
                th_0_min,       th_2_max;...
                th_0_center,    th_2_max;...
                th_0_max,       th_2_max].';
            
% % set equilibrium points (worst case)
% th_2_center = (th_2_min+th_2_max)/2; 
% th_0_center = (th_0_min+th_0_max)/2;
% q_e_list = [    th_0_min,       th_2_min;...
%                 th_0_min,       th_2_max;...
%                 th_0_max,       th_2_min;...
%                 th_0_max,       th_2_max;...
%                 th_0_center,    th_2_center].';

% % set equilibrium points
% q_e_list = [th_0_min,       (th_0_min+th_0_max)/2,  th_0_max;...
%             th_2_center,    th_2_center,            th_2_center];
[tmp,n_e] = size(q_e_list);


%% Lagrange equations
l1 = lAG;
l2 = lGJ;       % length from knee to hip
lcm2 = lDJ/2;   % length rotation point to center of mass (half of leg length)
l3 = lJL;       % length from hip to shoulder
lcm3 = lJL/2;
lFsp1 = lAE;
lFsp2 = lJK;

% % Constraints
% h1 = angle0_to_angle1(th0)-th1;
% W = [diff(h1,th0);diff(h1,th1)];

% Kinetic energy
T_m1 = 1/8*m1*l1^2*th0d^2;                                                  % Kinetic energy lower leg
xd_m2 = l1*th0d*cos(th0)+lcm2*th1d*cos(th1);                                % velocity x direction of mass 2
yd_m2 = l1*th0d*sin(th0)+lcm2*th1d*sin(th1);                                % velocity y direction of mass 2
T_m2 = (1/2*m2*(xd_m2^2+yd_m2^2));                                          % Kinetic energy upper leg
xd_m3 = l1*th0d*cos(th0)+l2*th1d*cos(th1)+lcm3*th2d*cos(th2);               
yd_m3 = l1*th0d*sin(th0)+l2*th1d*sin(th1)+lcm3*th2d*sin(th2);
T_m3 = (1/2*m3*(xd_m3^2+yd_m3^2));                                          % Kinetic energy trunk
xd_m4 = l1*th0d*cos(th0)+l2*th1d*cos(th1)+l3*th2d*cos(th2);
yd_m4 = l1*th0d*sin(th0)+l2*th1d*sin(th1)+l3*th2d*sin(th2);
T_m4 = (1/2*m4*(xd_m4^2+yd_m4^2));                                          % Kinetic energy arms
T = T_m1+T_m2+T_m3+T_m4;                                                    % Kinetic energy total

% Potential energy
V_RL = 1/2*m1*g*l1*sin(th0);                                    % Gravity rear leg
V_UL = m2*g*(l1*sin(th0)+lcm2*sin(th1-th0));                    % Gravity upper leg
V_T = m3*g*(l1*sin(th0)+l2*sin(th1-th0)+lcm3*sin(th2-th1+th0)); % Gravity Trunk
V_A = m4*g*(l1*sin(th0)+l2*sin(th1-th0)+l3*sin(th2-th1+th0));   % Gravity Arm
lspr1 = angle0_to_spring1(th0);                                 % length spring leg
V_spring1 = 0.5*N_leg*K_leg*(L0_leg-lspr1)^2;                   % Spring energy leg
lspr2 = angle2_to_spring2(th2);                                 % length spring trunk
V_spring2 = 0.5*N_trunk*K_trunk*(L0_trunk-lspr2)^2;             % Spring energy trunk
V = V_RL+V_UL+V_T+V_A+V_spring1+V_spring2;                          % Total

% Non conservative moments external
Qnc_ex = zeros(n_q,1);  Qnc_ex = sym(Qnc_ex);
lsp1 = angle0_to_spindle1(th0);         % length spindle leg
th_sp1 = cosinerule(lsp1,lAE,lAC,[]);   % angle of force applied on leg
Qnc_ex(1) = lFsp1*sin(th_sp1)*Fsp1;       % Applied moment on angle 0
Qnc_ex(2) = 0;                            % Applied moment on angle 1
lsp2 = angle2_to_spindle2(th2);         % length spindle trunk
th_sp2 = cosinerule(lsp2,lJK,lJH,[]);   % angle of force applied on trunk
Qnc_ex(3) = lFsp2*sin(th_sp2)*Fsp2;       % Applied moment on angle 2

% Non conservative moments internal
Qnc_in = zeros(n_q,1);  Qnc_in = sym(Qnc_in);
% % damping
Qnc_in(1) = Bdamp*th0d;
Qnc_in(2) = Bdamp*th1d;
Qnc_in(3) = Bdamp*th2d;

% d/dt*(T,qd)
Tqd = sym_partial_derivative(T,qd);
dtdTqd = sym_time_derivative(Tqd,[q;qd],[qd;qdd]);

% T,q
Tq = sym_partial_derivative(T,q);

% V,q
Vq = sym_partial_derivative(V,q);



%% Motion equations

M = sym_partial_derivative(dtdTqd.',qdd);
H_dtdTqd = sym_time_derivative(Tqd.',q,qd);
H = H_dtdTqd-Tq.' + Vq.' - Qnc_in;
S = sym_partial_derivative(Qnc_ex,tau);

%% Independant coordinates
% Tm 
Tm = sym_partial_derivative(q_qmin,q_min);
% k tilde, set to zero, because the partial derivative of q to time is zero
k_tilde = zeros(n_q,1);     k_tilde = sym(k_tilde);

% time derivative of Tm and k tilde
dTm = sym_time_derivative(Tm,[q_min;qd_min],[qd_min;qdd_min]);
dk_tilde = sym_time_derivative(k_tilde,[q_min;qd_min],[qd_min;qdd_min]);

% k bar
k_bar = dTm*qd_min+dk_tilde;

% centrallized matrices
Mc = subs(M,q,q_qmin);
Hc = subs(H,[q;qd],[q_qmin;Tm*qd_min+k_tilde]);
Sc = subs(S,q,q_qmin);% set 3 equilibrium points
th_e_list = [th_0_min;(th_0_min+th_0_max)/2;th_0_max];

% transposed used multiple times
TmT = Tm.';

% minimal system
M_min = TmT*Mc*Tm;
H_min = TmT*Hc+TmT*Mc*k_bar;
S_min = TmT*Sc;

%% Add motor model
R = r_ls.*diag([r_gear1, r_gear2]);
Jm = eye(2).*J_m;
Bm = eye(2).*B_m;

% compute theta motor dependant on theta joint
thm = [r_ls.*r_gear1.*angle0_to_spindle1(q_min(1));...
    r_ls.*r_gear2.*angle2_to_spindle2(q_min(2))];

% compute derivatives of theta motor to theta joints
dthm_dt = sym_partial_derivative(thm,q_min)*qd_min;                 % d(thm)/d(t)*qd
ddthmdt_dqd = sym_partial_derivative(dthm_dt,qd_min);               % d( d(thm)/d(t)*qd )/d(qd)
ddthmdt_dq_qd = sym_partial_derivative(dthm_dt,q_min)*qd_min;       % d(d(thm)/d(t)*qd)/dq *qd

SR_inv = inv(S_min*R);

% Compute the new system matrices
M_m = SR_inv*M_min+Jm*ddthmdt_dqd;
H_m = SR_inv*H_min+Jm*ddthmdt_dq_qd+Bm*dthm_dt;

S_m = eye(2).*Kelm*Km_m;


%% Motion -> Linearize -> State space -> Transfer function
leg_list_num = {};

% partial derivatives of H_min
dH_dqd = sym_partial_derivative(H_m,qd_min);
dH_dq = sym_partial_derivative(H_m,q_min);

% log tau equilibria
tau_e = cell(n_e,1);

% create figures
fig1 = figure; scr rt; 
% fig2 = figure; scr lt; fig3 = figure; scr lb;

% loop over equilibria
for i=1:1:n_e  
    % equilibrium points
    q_e = q_e_list(:,i);
    
    % equilibrium forces
    H_e = double(subs(H_m,[q_min;qd_min],[q_e;zeros(size(qd_min))]));
    S_e = double(subs(S_m,q_min,q_e));
    tau_e{i} = inv(S_e)*H_e;
    
    % derivative used for linearization
    Stau_e = S_m*tau_e{i};
    dS_dq = sym_partial_derivative(Stau_e,q_min);
    
    % Linearized matrices
    M_lin = double(subs(M_m,q_min,q_e));
    D_lin = double(subs(dH_dqd,[q_min;qd_min],[q_e;zeros(n_min,1)]));
    K_lin = double(subs(dH_dq,[q_min;qd_min],[q_e;zeros(n_min,1)]))-...
        double(subs(dS_dq,q_min,q_e));
    S_lin = double(subs(S_m,q_min,q_e));
    
    % inverse M
    invM_lin = inv(M_lin);
    
    % State space form
    A = [ zeros(size(M_lin)) eye(size(M_lin));
        -invM_lin*K_lin       -invM_lin*D_lin  ]; 
    B = [zeros(n_min) ; invM_lin*S_lin];
    
    % State space to double
    A = double(A);
    B = double(B);
    
    % state space system
    sys{i} = C*inv(eye(size(A))*s-A)*B+D;
%     sys{i} = sys{i}*P_ms;
    
    % get legend entries
    leg_list{i} = ['th_e ',num2str(q_e(1),2),', ',num2str(q_e(2),2),', tau_e ',num2str(double(tau_e{i}(1)),4),', ',num2str(double(tau_e{i}(2)),4)];
    
    
    figure(fig1);
    bode(sys{i},Bopt);
    legend(leg_list);
    hold all;
    
%     figure(fig2);
%     nyquist(sys{i}(1,1));
%     hold all;
%     figure(fig3);
%     nyquist(sys{i}(2,2));
%     hold all;
    
    disp(['Configuration th_e ',num2str(q_e(1),2),', ',num2str(q_e(2),2),', tau_e ',num2str(double(tau_e{i}(1)),4),', ',num2str(double(tau_e{i}(2)),4)]);
    % Stability check
    eigenvalues_A = eig(A)

    %% calculate eigenvalues
    [v,d] = eig(double(K_lin),double(M_lin));
    eigenmodes = v;
    eigenvalues = d;
    eigenfrequency_Hz = sqrt(diag(abs(d)))/2/pi % in Hz
    
        
    
end

%% Print equilibria
disp('Equilibria');
format_str = ' q0 = %5.3f, q2 = %5.3f, tau1 = %5.3f V, tau2 = %5.3f V.';
for i=1:1:n_e
    str = sprintf(format_str,q_e_list(:,i),tau_e{i});
    disp(str);
end

%% plot results per diagonal element
% close all
bod1 = figure; scr lt;
bod2 = figure; scr lb;
leg_list_num = {'1','2','3','4','5','6','7','8','9'};
line_type_list1 = {'bluea--0.5','blueb--0.5','bluec--0.5','greena--0.5','greenb--0.5','greenc--0.5','reda--0.5','redb--0.5','redc--0.5'};
line_type_list2 = {'bluea--0.5','greena--0.5','reda--0.5','blueb--0.5','greenb--0.5','redb--0.5','bluec--0.5','greenc--0.5','redc--0.5'};

frf_opt{1} = 1e-4; frf_opt{2} = 1e3; frf_opt{7} = 'on';
for i=1:1:n_e
    figure(bod1)
    frf(sys{i}(1,1),line_type_list2{i},frf_opt); hold all;
    figure(bod2)
    frf(sys{i}(2,2),line_type_list1{i},frf_opt); hold all;
end
figure(bod1); 
% ylim([-150 0]);
title('Voltage motor 1 to angle 0');
legend(leg_list,'location','eastoutside');

figure(bod2); 
% ylim([-150 0]);
title('Voltage motor 2 to angle 2');
legend(leg_list_num,'location','eastoutside');


% all_grids_on(); figure(bod1)
% save_report(bod1,'../../../Report/Images/Model','torso_mod_lin_1','bodeleg');
% save_report(bod2,'../../../Report/Images/Model','torso_mod_lin_2','bodeleg');


all_grids_on();
%% save system for analysis
% save('sys_','sys','q_e_list','tau_e')
% M = M_m; H = H_m; S = S_m; q = q_min; qd = qd_min; qdd = qdd_min;
% syms V1 V2
% tau = [V1;V2];
% save('motion_Volt_angle','M','H','S','q','qd','qdd','tau');

%% RGA analysis
% 
% fig_rga1 = figure; scr lt; fig_rga2 = figure; scr lb;
% for i = 1:1:n_e
%     omega = logspace(-2,2);
%     for j=1:length(omega)
%         Gf = freqresp(sys{i},omega(j));
%         RGAw(:,:,j) = Gf.*inv(Gf).';
%         RGAno(j) = sum(sum(abs(RGAw(:,:,j)-eye(2))));
%     end
%     RGA = frd(RGAw,omega);
%     figure(fig_rga1)
%     frfmag(RGA)
%     hold all;
%     figure(fig_rga2);
%     semilogx(omega,RGAno);
%     ylim([0 6])
%     hold all
%     
%     clear RGAw Gf RGAno RGA
%     
% end
% legend(leg_list)
% 
% 
% all_grids_on();
