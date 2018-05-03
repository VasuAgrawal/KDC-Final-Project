clear;
close all;
clc;

load('eoms.mat');

% load('data/run1.mat');

angles = randn(2000, 5);
velocities = randn(2000, 5);
accelerations = randn(2000, 5);
torques = randn(2000, 5);

A_ = [];
b_ = [];

n = [];
for i = 1:50:size(angles,1)
    angle = angles(i, :);
    vel = velocities(i, :);
    acc = accelerations(i, :);
    torque = torques(i, :);
        
    params = struct;
    params.th1 = angle(1);
    params.th2 = angle(2);
    params.th3 = angle(3);
    params.th4 = angle(4);
    params.th5 = angle(5);
    params.th1_dot = vel(1);
    params.th2_dot = vel(2);
    params.th3_dot = vel(3);
    params.th4_dot = vel(4);
    params.th5_dot = vel(5);
    params.th1_ddot = acc(1);
    params.th2_ddot = acc(2);
    params.th3_ddot = acc(3);
    params.th4_ddot = acc(4);
    params.th5_ddot = acc(5);
    params.T1 = torque(1);
    params.T2 = torque(2);
    params.T3 = torque(3);
    params.T4 = torque(4);
    params.T5 = torque(5);
    
    A_sub = subs(A, params);
    b_sub = subs(b, params);
    
    A_ = [A_; double(A_sub)];
    b_ = [b_; double(b_sub)];
end

x = A_ \ b_;
