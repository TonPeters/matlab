clear all; close all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Determine ratios between angles and spindles
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

run torso_measures_NX

th_0_min = spindle1_to_angle0(min_spindle1);
th_0_max = spindle1_to_angle0(max_spindle1);
th_2_min = spindle2_to_angle2(min_spindle2);
th_2_max = spindle2_to_angle2(max_spindle2);

% drive train dynamics
r_gear1 = 5/2;          % rad/rad,      Gear ratio from spindle to motor 1
r_gear2 = 13/3;         % rad/rad,      Gear ratio from spindle to motor 2
l_ls    = 0.002;        % m,           Lead of the spindle
r_sp     = 2*pi/l_ls;   % rad/mm,       Gear ratio from spindle translation to rotation

%% angle 1 dependant on angle 0
th0 = linspace(th_0_min,th_0_max).';
th1_th0 = angle0_to_angle1(th0);
Fth1_th0 = fit(th0,th1_th0,'poly1');
r_th1_th0 = Fth1_th0.p1;

figure;
plot(th0,th1_th0,th0,feval(Fth1_th0,th0));
xlabel('anlge 0'); ylabel('angle 1'); legend('exact','fit');

%% change in angle1 dependant on angle 0
syms q0_sym
r_sym = sym_partial_derivative(angle0_to_angle1(q0_sym),q0_sym);
dth1_th0 = double(subs(r_sym,q0_sym,th0));        % gear ratio joint 1 to joint 2
Fdth1_th0 = fit(th0,dth1_th0,'poly4');

figure;
plot(th0,dth1_th0,th0,feval(Fdth1_th0,th0));
xlabel('anlge 0'); ylabel('diff angle 1'); legend('exact','fit');

%% spindle 1 dependant on angle 0
sp1_th0 = angle0_to_spindle1(th0);
Fsp1_th0 = fit(th0,sp1_th0,'poly1');
r_sp1_th0 = Fsp1_th0.p1;

figure;
plot(th0,sp1_th0,th0,feval(Fsp1_th0,th0));
xlabel('anlge 0'); ylabel('spindle 1'); legend('exact','fit');

%% motor 1 dependant on angle 0
m1_th0 = sp1_th0*r_sp*r_gear1;
Fm1_th0 = fit(th0,m1_th0,'poly1');
r_m1_th0 = Fm1_th0.p1;

figure;
plot(th0,m1_th0,th0,feval(Fm1_th0,th0));
xlabel('anlge 0'); ylabel('motor 1'); legend('exact','fit');

%% Force angle 1 dependant on angle 0
thF1_th0 = cosinerule(di(A,E),sp1_th0,di(A,C),[]);
FthF1_th0 = fit(th0,thF1_th0,'poly1');
r_thF1_th0 = FthF1_th0.p1;

figure;
plot(th0,thF1_th0,th0,feval(FthF1_th0,th0));
xlabel('anlge 0'); ylabel('Force 1 angle'); legend('exact','fit');

%% Force reduction dependant on angle 0
rF1_th0 = sin(thF1_th0);
FrF1_th0 = fit(th0,rF1_th0,'poly1');
r_rF1_th0 = FrF1_th0.p1;

figure;
plot(th0,rF1_th0,th0,feval(FrF1_th0,th0));
xlabel('anlge 0'); ylabel('Reduction force 1'); legend('exact','fit');

%% spindle 2 dependant on angle 2
th2 = linspace(th_2_min,th_2_max).';
sp2_th2 = angle2_to_spindle2(th2);
Fsp2_th2 = fit(th2,sp2_th2,'poly1');
r_sp2_th2 = Fsp2_th2.p1;

figure;
plot(th2,sp2_th2,th2,feval(Fsp2_th2,th2));
xlabel('anlge 2'); ylabel('spindle 2'); legend('exact','fit');

%% motor 1 dependant on angle 0
m2_th2 = sp2_th2*r_sp*r_gear2;
Fm2_th2 = fit(th2,m2_th2,'poly1');
r_m2_th2 = Fm2_th2.p1;

figure;
plot(th2,m2_th2,th2,feval(Fm2_th2,th2));
xlabel('anlge 2'); ylabel('motor 2'); legend('exact','fit');

%% Force angle 2 dependant on angle 2
thF2_th2 = cosinerule(di(J,K),sp2_th2,di(H,J),[]);
FthF2_th2 = fit(th2,thF2_th2,'poly1');
r_thF2_th2 = FthF2_th2.p1;

figure;
plot(th2,thF2_th2,th2,feval(FthF2_th2,th2));
xlabel('anlge 2'); ylabel('Force 2 angle'); legend('exact','fit');

%% Force reduction dependant on angle 2
rF2_th2 = sin(thF2_th2);
FrF2_th2 = fit(th2,rF2_th2,'poly1');
FrF2_th2_p2 = fit(th2,rF2_th2,'poly2');
r_rF2_th2 = FrF2_th2.p1;

figure;
plot(th2,rF2_th2,th2,feval(FrF2_th2,th2),th2,feval(FrF2_th2_p2,th2));
xlabel('anlge 2'); ylabel('Reduction force 2'); legend('exact','fit1','fit2');

all_grids_on();