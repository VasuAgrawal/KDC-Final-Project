close all
clear all
clc

q1 = [0 0 72]';
q2 = [0 -33 147]';
q3 = [0 -33 302]';
q4 = [0 -33 437]';
q5 = [0 -33 518]';

w1 = [0 0 -1]';
w2 = [1 0 0]';
w3 = [1 0 0]';
w4 = [1 0 0]';
w5 = [0 0 -1]';

xi_1 = [-cross(w1,q1); w1];
xi_2 = [-cross(w2,q2); w2];
xi_3 = [-cross(w3,q3); w3];
xi_4 = [-cross(w4,q4); w4];
xi_5 = [-cross(w5,q5); w5];

xi = [xi_1 xi_2 xi_3 xi_4 xi_5];

th_start = [0 0 0 0 0]';
targ_pos = [57 -36 512]'

joint_traj = DLS_Traj(th_start,xi,q5,targ_pos);
%joint_traj = psuedoInvTraj(th_start,xi,q5,targ_pos);

%checking IK final position
n = length(th_start);
qlast = q5;
gslee_0 = [eye(3) qlast; [0 0 0] 1];

th = joint_traj(:,end);

gslee_th = eye(4);
for i = 1:n
    gslee_th = gslee_th * expm(wedge(xi(:,i))*th(i));
end
gslee_th = gslee_th * gslee_0;

final_pos = gslee_th(1:3,4)

x_axis = 1:length(joint_traj(1,:));

%plot
figure();
hold on
plot(x_axis,joint_traj(1,:));
plot(x_axis,joint_traj(2,:));
plot(x_axis,joint_traj(3,:));
plot(x_axis,joint_traj(4,:));
plot(x_axis,joint_traj(5,:));
xlabel('timestep #');
ylabel('joint value');
title('IK test');
legend('joint 1','joint 2','joint 3','joint 4','joint 5');
hold off
