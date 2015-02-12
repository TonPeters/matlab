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

%% angle 1 dependant on angle 0
th0 = linspace(th_0_min,th_0_max).';
th1_th0 = angle0_to_angle1(th0);
Fth1_th0 = fit(th0,th1_th0,'poly1');
r_th1_th0 = Fth1_th0.p1;

figure;
plot(th0,th1_th0,th0,feval(Fth1_th0,th0));
xlabel('anlge 0'); ylabel('angle 1'); legend('exact','fit');

%% spindle 1 dependant on angle 0
sp1_th0 = angle0_to_spindle1(th0);
Fsp1_th0 = fit(th0,sp1_th0,'poly1');
r_sp1_th0 = Fsp1_th0.p1;

figure;
plot(th0,sp1_th0,th0,feval(Fsp1_th0,th0));
xlabel('anlge 0'); ylabel('spindle 1'); legend('exact','fit');

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
xlabel('anlge 2'); ylabel('Reduction force 2'); legend('exact','fit');

all_grids_on();