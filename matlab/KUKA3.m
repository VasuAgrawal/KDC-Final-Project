clear;
close all;
clc;

%% Step 1 - Compute overall manipulator inertia matrix M

% Define symbolics
syms th1 th2 th3 th4 th5 'real';

% Determine vectors for rotation (in the base frame)
w1 = [0 0 -1]';
w2 = [1 0 0]';
w3 = [1 0 0]';
w4 = [1 0 0]';
w5 = [0 0 -1]';

% Determine points along each axis (again in the base frame)
q1 = [0 0 0.072]';
q2 = [0 -0.033 0.147]';
q3 = [0 -0.033 0.302]';
q4 = [0 -0.033 0.437]';
q5 = [0 -0.033 0.518]';

% Center of mass offsets of the joints
syms o1x o2x o3x o4x o5x 'real';
syms o1y o2y o3y o4y o5y 'real';
syms o1z o2z o3z o4z o5z 'real';
com_offset_1 = [o1x o1y o1z]';
com_offset_2 = [o2x o2y o2z]';
com_offset_3 = [o3x o3y o3z]';
com_offset_4 = [o4x o4y o4z]';
com_offset_5 = [o5x o5y o5z]';

% Update these points to be points at the center of mass of the link
p1_com = q1 + com_offset_1;
p2_com = q2 + com_offset_2;
p3_com = q3 + com_offset_3;
p4_com = q4 + com_offset_4;
p5_com = q5 + com_offset_5;

% Define the twists
xi_1 = [-cross(w1,q1); w1];
xi_2 = [-cross(w2,q2); w2];
xi_3 = [-cross(w3,q3); w3];
xi_4 = [-cross(w4,q4); w4];
xi_5 = [-cross(w5,q5); w5];

% Define forward kinematics - using the center of mass frames
gsl1_0 = [eye(3) p1_com; [0 0 0] 1];
gsl2_0 = [eye(3) p2_com; [0 0 0] 1];
gsl3_0 = [eye(3) p3_com; [0 0 0] 1];
gsl4_0 = [eye(3) p4_com; [0 0 0] 1];
gsl5_0 = [eye(3) p5_com; [0 0 0] 1];

% Derove Jacobians
J1 = [(adj(expm(wedge(xi_1)*th1) * gsl1_0)) \ xi_1, zeros(6, 4)];
J2 = [(adj(expm(wedge(xi_1)*th1) * expm(wedge(xi_2)*th2) * gsl2_0)) \ xi_1, ...
      (adj(expm(wedge(xi_2)*th2) * gsl2_0)) \ xi_2, ...
      zeros(6, 3)];
J3 = [(adj(expm(wedge(xi_1)*th1) * expm(wedge(xi_2)*th2) * expm(wedge(xi_3)*th3) * gsl3_0)) \ xi_1, ...
      (adj(expm(wedge(xi_2)*th2) * expm(wedge(xi_3)*th3) * gsl3_0)) \ xi_2, ...
      (adj(expm(wedge(xi_3)*th3) * gsl3_0)) \ xi_3, ...
      zeros(6, 2)];
J4 = [(adj(expm(wedge(xi_1)*th1) * expm(wedge(xi_2)*th2) * expm(wedge(xi_3)*th3) * expm(wedge(xi_4)*th4) * gsl4_0)) \ xi_1, ...
      (adj(expm(wedge(xi_2)*th2) * expm(wedge(xi_3)*th3) * expm(wedge(xi_4)*th4) * gsl4_0)) \ xi_2, ...
      (adj(expm(wedge(xi_3)*th3) * expm(wedge(xi_4)*th4) * gsl4_0)) \ xi_3, ...
      (adj(expm(wedge(xi_4)*th4) * gsl4_0)) \ xi_4, ...
      zeros(6, 1)];
J5 = [(adj(expm(wedge(xi_1)*th1) * expm(wedge(xi_2)*th2) * expm(wedge(xi_3)*th3) * expm(wedge(xi_4)*th4) * expm(wedge(xi_5)*th5) * gsl5_0)) \ xi_1, ...
      (adj(expm(wedge(xi_2)*th2) * expm(wedge(xi_3)*th3) * expm(wedge(xi_4)*th4) * expm(wedge(xi_5)*th5) * gsl5_0)) \ xi_2, ...
      (adj(expm(wedge(xi_3)*th3) * expm(wedge(xi_4)*th4) * expm(wedge(xi_5)*th5) * gsl5_0)) \ xi_3, ...
      (adj(expm(wedge(xi_4)*th4) * expm(wedge(xi_5)*th5) * gsl5_0)) \ xi_4, ...
      (adj(expm(wedge(xi_5)*th5) * gsl5_0)) \ xi_5];
  
J1 = simplify(J1);
J2 = simplify(J2);
J3 = simplify(J3);
J4 = simplify(J4);
J5 = simplify(J5);

% Parameters of links
syms m1 m2 m3 m4 m5 'real';
syms Ix1 Iy1 Iz1 'real';
syms Ix2 Iy2 Iz2 'real';
syms Ix3 Iy3 Iz3 'real';
syms Ix4 Iy4 Iz4 'real';
syms Ix5 Iy5 Iz5 'real';

M_1 = diag([m1 m1 m1 Ix1 Iy1 Iz1]);
M_2 = diag([m2 m2 m2 Ix2 Iy2 Iz2]);
M_3 = diag([m3 m3 m3 Ix3 Iy3 Iz3]);
M_4 = diag([m4 m4 m4 Ix4 Iy4 Iz4]);
M_5 = diag([m5 m5 m5 Ix5 Iy5 Iz5]);

% Derive the overall manipulator inertia matrix M
M = J1'*M_1*J1 + J2'*M_2*J2 + J3'*M_3*J3 + J4'*M_4*J4 + J5'*M_5*J5;
M = simplify(M);
fprintf('Overall Mass Matrix \n');
disp(M);

%% Step 2 - Compute manipulator Coriolis matrix C
syms th1_dot th2_dot th3_dot th4_dot th5_dot 'real';
syms th1_ddot th2_ddot th3_ddot th4_ddot th5_ddot 'real';
th = [th1 th2 th3 th4 th5];
th_dot = [th1_dot th2_dot th3_dot th4_dot th5_dot];
th_ddot = [th1_ddot th2_ddot th3_ddot th4_ddot th5_ddot];

n = numel(th);
C = sym(zeros(n));

% Add elements to the C matrix per the formula. Make sure to simplify
for i = 1:n
    for j = 1:n
        for k = 1:n
            add = diff(M(i, j), th(k)) + diff(M(i, k), th(j)) - diff(M(k, j), th(i));
            add = simplify(add);
            add = add * th_dot(k);
            C(i, j) = C(i, j) + 0.5 * add;
        end
    end
end

C = simplify(C);
fprintf('Overall Coriolis Matrix \n');
disp(C);

%% Step 3 - Compute N, the potential energy matrix

% Come up with the forward kinematics to each frame
gsl1_th = expm(wedge(xi_1) * th1) * gsl1_0;
gsl2_th = expm(wedge(xi_1) * th1) * expm(wedge(xi_2) * th2) * gsl2_0;
gsl3_th = expm(wedge(xi_1) * th1) * expm(wedge(xi_2) * th2) * expm(wedge(xi_3) * th3) * gsl3_0;
gsl4_th = expm(wedge(xi_1) * th1) * expm(wedge(xi_2) * th2) * expm(wedge(xi_3) * th3) * expm(wedge(xi_4) * th4) * gsl4_0;
gsl5_th = expm(wedge(xi_1) * th1) * expm(wedge(xi_2) * th2) * expm(wedge(xi_3) * th3) * expm(wedge(xi_4) * th4) * expm(wedge(xi_5) * th5) * gsl5_0;

% Get the positions of each frame in world coordinates
p = [0 0 0 1]';
L1 = simplify(gsl1_th * p);
L2 = simplify(gsl2_th * p);
L3 = simplify(gsl3_th * p);
L4 = simplify(gsl4_th * p);
L5 = simplify(gsl5_th * p);

% Extract the z component
h1 = L1(3);
h2 = L2(3);
h3 = L3(3);
h4 = L4(3);
h5 = L5(3);

% Define v in terms of gravity
g = 9.81;
V = m1*g*h1 + m2*g*h2 + m3*g*h3 + m4*g*h4 + m5*g*h5;

% Solve for N
N = [diff(V, th1);
     diff(V, th2);
     diff(V, th3);
     diff(V, th4);
     diff(V, th5)];
 
N = simplify(N);
fprintf('Overall Normal Matrix \n');
disp(N);

%% Step 4 - Compute The Equations of Motion

syms T1 T2 T3 T4 T5 'real';
T = [T1 T2 T3 T4 T5];

EoM = M * th_ddot' + C * th_dot' + N;
eqns = EoM == T';
eqns = simplify(eqns);
save('eoms.mat');

%% Step 5 - Substitute in the nonlinear parameters into the equations

params = struct;

params.m1 = 1.390; %kg
params.m2 = 1.318; %kg
params.m3 = 0.821; %kg
params.m4 = 0.769; %kg
params.m5 = 0.687 + 0.199 + 0.010 + 0.010; %kg

mass_sub = subs(eqns, params);

syms u1x u2x u3x u4x u5x 'real';
syms u1y u2y u3y u4y u5y 'real';
syms u1z u2z u3z u4z u5z 'real';
square_sub = expand(mass_sub);

square_sub = subs(square_sub, o1x^2, u1x);
square_sub = subs(square_sub, o2x^2, u2x);
square_sub = subs(square_sub, o3x^2, u3x);
square_sub = subs(square_sub, o4x^2, u4x);
square_sub = subs(square_sub, o5x^2, u5x);

square_sub = subs(square_sub, o1y^2, u1y);
square_sub = subs(square_sub, o2y^2, u2y);
square_sub = subs(square_sub, o3y^2, u3y);
square_sub = subs(square_sub, o4y^2, u4y);
square_sub = subs(square_sub, o5y^2, u5y);

square_sub = subs(square_sub, o1z^2, u1z);
square_sub = subs(square_sub, o2z^2, u2z);
square_sub = subs(square_sub, o3z^2, u3z);
square_sub = subs(square_sub, o4z^2, u4z);
square_sub = subs(square_sub, o5z^2, u5z);

syms u1xy u1yz u1xz 'real';
syms u2xy u2yz u2xz 'real';
syms u3xy u3yz u3xz 'real';
syms u4xy u4yz u4xz 'real';
syms u5xy u5yz u5xz 'real';
cross_sub = expand(square_sub);

cross_sub = subs(cross_sub, o1x*o1y, u1xy);
cross_sub = subs(cross_sub, o1y*o1z, u1yz);
cross_sub = subs(cross_sub, o1x*o1z, u1xz);

cross_sub = subs(cross_sub, o2x*o2y, u2xy);
cross_sub = subs(cross_sub, o2y*o2z, u2yz);
cross_sub = subs(cross_sub, o2x*o2z, u2xz);

cross_sub = subs(cross_sub, o3x*o3y, u3xy);
cross_sub = subs(cross_sub, o3y*o3z, u3yz);
cross_sub = subs(cross_sub, o3x*o3z, u3xz);

cross_sub = subs(cross_sub, o4x*o4y, u4xy);
cross_sub = subs(cross_sub, o4y*o4z, u4yz);
cross_sub = subs(cross_sub, o4x*o4z, u4xz);

cross_sub = subs(cross_sub, o5x*o5y, u5xy);
cross_sub = subs(cross_sub, o5y*o5z, u5yz);
cross_sub = subs(cross_sub, o5x*o5z, u5xz);

params = struct;
params.th1 = 1;
params.th2 = 1;
params.th3 = 1;
params.th4 = 1;
params.th5 = 1;
params.th1_dot = 1;
params.th2_dot = 1;
params.th3_dot = 1;
params.th4_dot = 1;
params.th5_dot = 1;
params.th1_ddot = 1;
params.th2_ddot = 1;
params.th3_ddot = 1;
params.th4_ddot = 1;
params.th5_ddot = 1;
params.T1 = 1;
params.T2 = 1;
params.T3 = 1;
params.T4 = 1;
params.T5 = 1;

th_sub = subs(cross_sub, params);

vars = symvar(th_sub);

[A, b] = equationsToMatrix(cross_sub, vars);
save('eoms.mat');