clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fit gravity compensation %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pos_l = {'Up','C','Do'};
p1 = zeros(3,1);
p2 = p1;

load motion_Volt_angle.mat

fig = figure; scr rt; 
fig2 = figure; scr lt;
for i=1:1:3
    pos = pos_l{i};
    load(['data/static_leg',pos,'_120s.mat']);
    if strcmp(pos,'Up')
        st1 = 5; st2 = 55; en1 = 48; en2 = 100;
    elseif strcmp(pos,'C')
        st1 = 5; st2 = 55; en1 = 48; en2 = 100;
    elseif strcmp(pos,'Do')
        st1 = 5; st2 = 55; en1 = 48; en2 = 100;
    end


%     figure;
%     plot(time,u(:,2))
    indices1 = find(time>st1 & time<en1);
    indices2 = find(time>st2 & time<en2);

    F1 = fit(enc(indices1,2),u(indices1,2),'poly1');
    F2 = fit(enc(indices2,2),u(indices2,2),'poly1');

    %% plot fit
    figure(fig2);
    plot(enc(indices1,2),u(indices1,2),enc(indices2,2),u(indices2,2)); hold all; grid on;
    plot(enc(indices1,2),feval(F1,enc(indices1,2)),enc(indices2,2),feval(F2,enc(indices2,2)));

    %% combine fits

    p1(i) = (F1.p1+F2.p1)/2;
    p2(i) = (F1.p2+F2.p2)/2;
    plot(enc(indices1,2),p1(i).*enc(indices1,2)+p2(i))
    
    figure(fig);
    plot(enc(indices1,2),p1(i).*enc(indices1,2)+p2(i)); hold all; grid on;
    
    %% gravity by model
    xsp_min = min(enc(:,2));
    xsp_max = max(enc(:,2));
    xsp0_mean = mean(enc(:,1));

    xsp = linspace(xsp_min,xsp_max,10).';
    th = spindle2_to_angle2(xsp);
    th0 = spindle1_to_angle0(xsp0_mean);
    S_inv = inv(S);

    u_model = zeros(size(xsp));
    for i=1:1:10
        H_num = double(subs(H,[q;qd],[th0;th(i);0;0]));
        u_tmp = S_inv*H_num;
        u_model(i) = u_tmp(2);
    end

    figure(fig2); 
    plot(xsp,u_model,'--');
    

    
end


all_grids_on();




