clear all; 
% close all;
clc;
if (gcf==1)
    close all;
    figure(11); scr rt;
end
    

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
run torso_measures_NX

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

%% other parameters
% th_0_min = 0.0;
% th_0_max = 1.11;
% th_2_min = 0.2175;
% th_2_max = 3.2276;
th_0_min = 0.1;
th_0_max = 1.05;
th_2_min = 0.37;
th_2_max = 3.05;
th_1_min = angle0_to_angle1(th_0_min);
th_1_max = angle0_to_angle1(th_0_max);

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

% % set equilibrium points (center points)
% th_2_center = (th_2_min+th_2_max)/2; 
% th_0_center = (th_0_min+th_0_max)/2;
% q_e_list = [    th_0_max,       th_2_max].';
            
% % set equilibrium points (worst case)
th_2_center = (th_2_min+th_2_max)/2; 
th_0_center = (th_0_min+th_0_max)/2;
q_e_list = [    th_0_min,       th_2_min;...
                th_0_min,       th_2_center;...
                th_0_min,       th_2_max;...
                th_0_center,    th_2_min;...
                th_0_center,    th_2_center;...
                th_0_center,    th_2_max;...
                th_0_max,       th_2_min;...
                th_0_max,       th_2_center;...
                th_0_max,       th_2_max].';

% % set equilibrium points
% q_e_list = [th_0_min,       (th_0_min+th_0_max)/2,  th_0_max;...
%             th_2_center,    th_2_center,            th_2_center];
[tmp,n_e] = size(q_e_list);

%% switching paramters
% m1 = 0;
% m2 = 0;
% m3 = 0;
m4 = 0;
N_leg = 0;
N_trunk = 0;

% tau = [Fsp1;Fsp2];
% 
% min_co = [1;3];
% n_min = length(min_co);

% q_min = []; qd_min = []; qdd_min = [];
% for i=1:1:n_min
%     q_min = [q_min;q(i)];
%     qd_min = [qd_min;qd(i)];
%     qdd_min = [qdd_min;qd(i)];
% end
% 
% % q writen in minimal coordinates
% q_qmin = [q_min(1);th_1_max;th_2_max];
% q_e_list = [    th_0_max; 0; -th_0_max].';
% q_e_list = q_e_list(1:n_min,:);

C = [eye(n_min) zeros(n_min)];
D = [zeros(n_min)];

n_t = length(tau);
[tmp,n_e] = size(q_e_list);
%% Lagrange equations
l1 = lAG;
l2 = lGJ;       % length from knee to hip
lcm2 = l2-lDJ/2;    % length rotation point to center of mass (half of leg length)
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

% % Manually simplified formulas
% T_m2_2 = 1/2*m2*(l1^2*th0d^2+lcm2^2*th1d^2+...
%     2*l1*lcm2*th0d*th1d*cos(th0-th1));
% T_m3_2 = 1/2*m3*(l1^2*th0d^2+l2^2*th1d^2+lcm3^2*th2d^2+...
%     2*l1*l2*th0d*th1d*cos(th0-th1)+2*l1*lcm3*th0d*th2d*cos(th0-th2)+...
%     2*lcm3*l2*th2d*th1d*cos(th2-th1));
% T_m4_2 = 1/2*m4*(l1^2*th0d^2+l2^2*th1d^2+l3^2*th2d^2+...
%     2*l1*l2*th0d*th1d*cos(th0-th1)+2*l1*l3*th0d*th2d*cos(th0-th2)+...
%     2*l3*l2*th2d*th1d*cos(th2-th1));
% T = T_m1+T_m2+T_m3+T_m4;  


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
% fig1 = figure; scr rt; fig2 = figure; scr lt; fig3 = figure; scr lb;

% loop over equilibria
for i=1:1:n_e    
    % equilibrium points
    q_e = q_e_list(:,i);
    
    % equilibrium forces
    H_e = double(subs(H_min,[q_min;qd_min],[q_e;zeros(size(qd_min))]));
    S_e = double(subs(S_min,q_min,q_e));
    tau_e{i} = inv(S_e)*H_e;
    
    % derivative used for linearization
    Stau_e = S_min*tau_e{i};
    dS_dq = sym_partial_derivative(Stau_e,q_min);
    
    % Linearized matrices
    M_lin = double(subs(M_min,q_min,q_e));
    D_lin = double(subs(dH_dqd,[q_min;qd_min],[q_e;zeros(n_min,1)]));
    K_lin = double(subs(dH_dq,[q_min;qd_min],[q_e;zeros(n_min,1)]))-...
        double(subs(dS_dq,q_min,q_e));
    S_lin = double(subs(S_min,q_min,q_e));
    
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
    
    
    % get legend entries
    leg_list = get(findobj(gcf,'Type','axes','Tag','legend'),'String');
    % get legend entries
    leg_temp = '';
    for j=1:1:n_min
        leg_temp = [leg_temp,'th_e ',num2str(q_e(j),2),', tau_e ',num2str(double(tau_e{i}(j)),4),', '];
    end
    leg_list{length(leg_list)+1} = leg_temp;
    
    figure(gcf);
    bode(sys{i},Bopt);
    legend(leg_list);
    hold all;

    
    disp(['Configuration, ',leg_temp]); 
    % Stability check
    eigenvalues_A = eig(A);

    %% calculate eigenvalues
    [v,d] = eig(double(K_lin),double(M_lin));
    eigenmodes = v;
    eigenvalues = d;
    eigenfrequency = sqrt(diag(abs(d)))/2/pi; % in Hz
    for j=1:1:length(d)
        disp(['eigenfrequency ',num2str(j),' = ',num2str(real(eigenfrequency(1))),' with eigencolumn = ']);
        eigenmodes(:,j)
    end
    % check for proportional damping if UtDU is diagonal
    UtDU = eigenmodes'*D_lin*eigenmodes;
    disp('check for proportional damping or not: Proportional if U^T*D*U is diagonal')
    UtDU
    
        
    
end

%% save system
M = M_min; H = H_min; S = S_min; q = q_min; qd = qd_min; qdd = qdd_min;
save('motion_no_spring_no_arms','M','H','S','q','qd','qdd','tau');



all_grids_on();


