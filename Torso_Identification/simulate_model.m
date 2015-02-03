clear all; close all; clc;
%% simulate system

load reference_standup.mat


indices=find(rem(t,0.25)==0);

tr = t(indices);
qr = q(indices,:);
qdr = qd(indices,:);
qddr = qdd(indices,:);
n_t = length(tr);
tau = zeros(2,n_t);
% compute required force
for i = 1:1:n_t
    clc; i
    tau(:,i) = inverse_dynamics_no_arms(qr(i,:).',qdr(i,:).',qddr(i,:).');
end

figure;
subplot(2,1,1); plot(tr,tau(1,:).')
subplot(2,1,2); plot(tr,tau(2,:).')