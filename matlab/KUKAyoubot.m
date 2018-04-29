clear all
close all
clc

%% Parameters

num_frames = 5;
ths = zeros(1,num_frames);
ths(3) = pi()/2;

z1 = [0 0 1]';
z2 = [1 0 0]';
z3 = [1 0 0]';
z4 = [1 0 0]';
z5 = [0 0 1]';

Z = [z1 z2 z3 z4 z5];

%Must add aditional frame from floor to arm (because of base)
%all in mm
q1 = [0 0 0]';
q2 = [0 0 147]';
q3 = [0 0 302]';
q4 = [0 0 437]';
q5 = [0 0 518]'; %adjust to make tool frame
%May add aditional frame to convert to new tool frame

Q = [q1 q2 q3 q4 q5];

rotZ = eye(4);
trans = [eye(3) q5;
         0 0 0 1];    
gst0 = trans * rotZ;

p = [0 0 0 1]';

T = zeros(4,4,num_frames); %transforms for each frame
G = zeros(4,4,num_frames); %FK for each frame
t = zeros(6,num_frames); %list of twists

%% Calculate FK

for i = 1:num_frames
    T(:,:,i) = twistM(Z(:,i),Q(:,i));
    gst = eye(4);
    trans_i = [eye(3) Q(:,i);
               0 0 0 1];
    gst0_i = trans_i * rotZ;
    for j = 1:i
        gst = gst*expm(T(:,:,j)*ths(j));
    end
    gst = gst*gst0_i;
    G(:,:,i) = gst;
    t(:,i) = [T(1:3,4,i);Z(:,i)];
end

%% Calculate Jacobian

J = zeros(6,num_frames);
for i = 1:num_frames
    if i == 1
        J(:,i) = t(:,i);
    else
        gst = eye(4);
        for j = 1:i
            gst = gst*expm(T(:,:,j)*ths(j));
        end
        J(:,i) = Adj(gst)*t(:,i);
    end
end

%% Plot FK

plotFK(num_frames,G);

%% Inverse Kinematics
% x = [x y z q qx qy qz]

% thd = [0 0 0 0 0];
% xd = FK(thd,T,gst0);
% xs = FK(ths,T,gst0);
% traj = pseudoinv(xd,xs,ths,T,gst0);

%% Helper Functions

function [traj,xtraj] = pseudoinv(xd,xs,th,T,gst0)
error = .01;
itLim = 1000;
tstep = .01;

num_frames = length(th);
traj = [th'];
xtraj = [xs];
e = QuatSub(xd,xs);
count = 0;

while and(not(sum(e>error)==0),(count<itLim))
    count = count+1;
    J = Jaco(th);
    Jdag = J'*inv(J*J');
    deltaTh = Jdag*e;
    th = th+deltaTh*tstep;
    traj = [traj th'];
    ee = FK(th,T,gst0);
    xtraj = [xtraj ee];
    e = QuatSub(xd,ee);
    e(1:3) = e(1:3) + cross(ee(1:3),e(4:6));
end

figure()
hold on
plot3(xtraj(1,:),xtraj(2,:),xtraj(3,:))
xlabel('x');
ylabel('y');
zlabel('z');
title('Position Trajectory');
hold off

time = 0:tstep:tstep*(length(traj(1,:))-1);

figure();
hold on
for i = 1:num_frames
    plot(time,traj(i,:));
end
legend('1','2','3','4','5');
xlabel('s');
ylabel('radians');
title('Joint Trajectory');
hold off

%{
%Trying to animate the trajectory, not there yet
figure();
hold on
for i = time
    [pos,G] = FK(traj,T,gst0);
    anim = plotFK(num_frames,G);
    pause(tstep);
    delete(anim);
end
%}
end

function [] = plotFK(n_frames,G)
X = zeros(1,n_frames);
Y = zeros(1,n_frames);
V = zeros(1,n_frames);

for i = 1:n_frames
    X(i) = G(1,4,i);
    Y(i) = G(2,4,i);
    V(i) = G(3,4,i);
end

figure()
hold on
for i = 1:n_frames-1
    plot3(X(i:i+1),Y(i:i+1),V(i:i+1));
end
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
legend('Link 1','Link 2','Link 3','Link 4');
title('KUKA Forward Kinematics');
hold off
end

function [pos,G] = FK(ths,T,gst0)
num_frames = length(ths);

p = [0 0 0 1]';

G = zeros(4,4); %FK for each frame

gst = eye(4);
for j = 1:num_frames
    gst = gst*expm(T(:,:,j)*ths(j));
end
gst = gst*gst0;
G = gst;
pt = G*p;
quat = -rotm2quat(G(1:3,1:3))';
pos = [pt(1:3);quat(2:4);quat(1)];
end

function J = Jaco(ths)
num_frames = length(ths);

z1 = [0 0 1]';
z2 = [1 0 0]';
z3 = [1 0 0]';
z4 = [1 0 0]';
z5 = [0 0 1]';

Z = [z1 z2 z3 z4 z5];

%Must add aditional frame from floor to arm (because of base)
%all in mm
q1 = [0 0 0]';
q2 = [0 0 147]';
q3 = [0 0 302]';
q4 = [0 0 437]';
q5 = [0 0 518]'; %adjust to make tool frame
%May add aditional frame to convert to new tool frame

Q = [q1 q2 q3 q4 q5];

T = zeros(4,4,num_frames); %transforms for each frame
t = zeros(6,num_frames); %list of twists

for i = 1:num_frames
    T(:,:,i) = twistM(Z(:,i),Q(:,i));
    t(:,i) = [T(1:3,4,i);Z(:,i)];
end

J = zeros(6,num_frames);
for i = 1:num_frames
    if i == 1
        J(:,i) = t(:,i);
    else
        gst = eye(4);
        for j = 1:i-1
            gst = gst*expm(T(:,:,j)*ths(j));
        end
        J(:,i) = Adj(gst)*t(:,i);
    end
end
end

function M = Adj(g)
R = g(1:3,1:3);
p = skew(g(1:3,4));
M = [R p*R; zeros(3,3) R];
end

function e = QuatSub(xd,xs)
%[x y z qx qy qz q]
e = zeros(1,6)';
e(1:3) = xd(1:3)-xs(1:3);
qd = [xd(7);xd(4:6)]';
qsinv = [xs(7);-xs(4:6)]';
qinv = qmult(qd,qsinv);
e(4:6) = qinv(2:4);
end

function c = qmult(a,b)
%[q qx qy qz]
c1 = a(1)*b(1)-a(2:4)*b(2:4)';
c = zeros(1,3);
c(1) = a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3);
c(2) = a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2);
c(3) = a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1);
c = [c1,c];
end