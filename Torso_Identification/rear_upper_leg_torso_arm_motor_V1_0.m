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

% clear parameters
clearvars -except m1 m2 m3 m4 lAG lAE lAC lGJ lJK lJH lDJ lJL aBAZ K_leg L0_leg N_leg K_trunk L0_trunk N_trunk Bdamp
%% motor parameters
R_m = 0.583;                % ohm
L_m = 0.191e-3;             % H
Kt_m = 29.2e-3;             % Nm/A,     torque constant
Ke_m = 1/(328*2*pi/60);     % V/(rad/s) Back Emf constant (1/speed constant)
J_m = 79.2/1000/10000;      % kg*m^2
b_m = 1/(6.55*2*pi/60*1000);% Nm/(rad/s) motor friction
l_ls = 0.002;               % m,        lead of the led screw
d_ls = 0.012;               % m,        diameter led screw
r_gear1 = 5/2;              % spindle/motor
r_gear2 = 13/3;             % spindle/motor

% motor & spindle model
s = tf('s');
P_ms1 = 2*pi/l_ls/r_gear1*(Kt_m*(J_m*s+b_m))/((L_m*s+R_m)*(J_m*s+b_m)+Ke_m*Kt_m);
P_ms2 = 2*pi/l_ls/r_gear2*(Kt_m*(J_m*s+b_m))/((L_m*s+R_m)*(J_m*s+b_m)+Ke_m*Kt_m);
P_ms = [P_ms1 0; 0 P_ms2];

%% Motor caracteristics
H1 = 1/(L_m*s+R_m)*Kt_m;
H2 = 1/(J_m*s+b_m);
H3 = 1/s;
H4 = Ke_m;
rk = r_gear1;


%% other parameters
% th_0_min = 0.0;
% th_0_max = 1.11;
% th_2_min = 0.2175;
% th_2_max = 3.2276;
th_0_min = 0.1;
th_0_max = 1.05;
th_2_min = 0.37;
th_2_max = 3.05;

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
T_RL = 1/8*m1*l1^2*th0d^2;                                                  % Kinetic energy lower leg
T_UL = 1/2*m2*(l1^2+lcm2^2-2*l1*lcm2*cos(th1))*th0d^2+1/2*m2*lcm2^2*th1d^2; % Kinetic energy upper leg
l_th1_cm3 = cosinerule(lcm3,l2,[],th2);                                         % length knee joint to CM3
l_th0_cm3 = cosinerule(l1,l_th1_cm3,[],th1+cosinerule(l2,l_th1_cm3,lcm3,[]));           % length ankle joint to CM3
T_T = 1/2*m3*(lcm3^2*th2d^2+l_th1_cm3^2*th1d^2+l_th0_cm3^2*th0d^2);                 % Kinetic energy trunk
l_th1_cm4 = cosinerule(l3,l2,[],th2);                                         % length knee joint to CM4
l_th0_cm4 = cosinerule(l1,l_th1_cm4,[],th1+cosinerule(l2,l_th1_cm4,l3,[]));           % length ankle joint to CM4
T_A = 1/2*m4*(l3^2*th2d^2+l_th1_cm4^2*th1d^2+l_th0_cm4^2*th0d^2);                 % Kinetic energy arms
T = T_RL+T_UL+T_T+T_A;                                                          % Total

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


%% Motion -> Linearize -> State space -> Transfer function
leg_list = {};

% partial derivatives of H_min
dH_dqd = sym_partial_derivative(H_min,qd_min);
dH_dq = sym_partial_derivative(H_min,q_min);

% log tau equilibria
tau_e = cell(n_e,1);

% create figures
fig1 = figure; scr rt; fig2 = figure; scr lt; fig3 = figure; scr lb;

% loop over equilibria
% for i=1:1:n_e  
for i=5
    % equilibrium points
    q_e = q_e_list(:,i);
    
    % equilibrium forces
    H_e = subs(H_min,[q_min;qd_min],[q_e;zeros(size(qd_min))]);
    S_e = subs(S_min,q_min,q_e);
    tau_e{i} = inv(S_e)*H_e;
    
    % derivative used for linearization
    Stau_e = S_min*tau_e{i};
    dS_dq = sym_partial_derivative(Stau_e,q_min);
    
    % Linearized matrices
    M_lin = subs(M_min,q_min,q_e);
    D_lin = subs(dH_dqd,[q_min;qd_min],[q_e;zeros(n_min,1)]);
    K_lin = subs(dH_dq,[q_min;qd_min],[q_e;zeros(n_min,1)])-...
        subs(dS_dq,q_min,q_e);
    S_lin = subs(S_min,q_min,q_e);
    
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
%     legend(leg_list);
    hold all;
    
    figure(fig2);
    nyquist(sys{i}(1,1));
    hold all;
    figure(fig3);
    nyquist(sys{i}(2,2));
    hold all;
    
    disp(['Configuration th_e ',num2str(q_e(1),2),', ',num2str(q_e(2),2),', tau_e ',num2str(double(tau_e{i}(1)),4),', ',num2str(double(tau_e{i}(2)),4)]);
    % Stability check
    eigenvalues_A = eig(A)

    %% calculate eigenvalues
    [v,d] = eig(double(K_lin),double(M_lin));
    eigenmodes = v;
    eigenvalues = d;
    eigenfrequency_Hz = sqrt(diag(abs(d)))/2/pi % in Hz
    
        
    
end




%% add motor
lead = 0.002;
d0 = 0.012;
% Gs = angle0_to_spindle1(sys{i}(1,1)*2*pi/lead)/lead*2*pi;

Gs_inv = Gs;
Plant = (1/rk*H3*H2*H1+1/rk*H3*H2*H1*H2*H1/(1+H2*H1*H4))/(1+1/rk^2*H3*H2*Gs_inv-1/rk^2*H3*H2*H1*H2*Gs_inv/(1+H2*H1*H4));
Plant2 = (1/rk*H3*H2*H1/(1+H1*H4*H2))/(1+1/rk^2*H3*H2*Gs_inv/(1+H1*H4*H2));

figure; bode(Plant2)

%% bode motor
B_m = b_m;
P_V_to_thetam = Kt_m/(s*((L_m*s+R_m)*(J_m*s+B_m)+Ke_m*Kt_m));
figure
bode(P_V_to_thetam)



%% save system for analysis
% save('system_complete_9','sys','q_e_list','tau_e')
% M = M_min; H = H_min; S = S_min; q = q_min; qd = qd_min; qdd = qdd_min;
% save('motion_complete','M','H','S','q','qd','qdd','tau');




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





all_grids_on();


