% % Note: all dimensions are in millimeters
% 
% % Step 1: Determine the transform from base frame to the tool frame.
% % Base frame: Center of the arm, y pointing forward.
% t = [0, -33, 608]'; % Tool frame is base of the gripper
% R = eye(3);
% gst0 = [R, t;
%         0 0 0 1];
% 
% % Determine vectors for rotation (in the base frame)
% w1 = [0 0 -1]';
% w2 = [1 0 0]';
% w3 = [1 0 0]';
% w4 = [1 0 0]';
% w5 = [0 0 -1]';
% 
% % Determine points along each axis (again in the base frame)
% q1 = [0 0 72]';
% q2 = [0 -33 147]';
% q3 = [0 -33 302]';
% q4 = [0 -33 437]';
% q5 = [0 -33 518]';
% 
% % Twist matrices
% t1 = twistM(w1, q1);
% t2 = twistM(w2, q2);
% t3 = twistM(w3, q3);
% t4 = twistM(w4, q4);
% t5 = twistM(w5, q5);
% 
% % Test point, at the tip of the gripper
% gripper_tip = [0 0 47 1]';
% 
% th = [0 0 0 0 0];
% gst = expm(t1 * th(1)) * expm(t2 * th(2)) * ...
%       expm(t3 * th(3)) * expm(t4 * th(4)) * ...
%       expm(t5 * th(5)) * gst0;
% tip = gst * gripper_tip;
% disp(tip(1:3));

%% Generate EOMs for the KUKA arm

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
q1 = [0 0 72]';
q2 = [0 -33 147]';
q3 = [0 -33 302]';
q4 = [0 -33 437]';
q5 = [0 -33 518]';

% Center of mass offsets of the joints
com_offset_1 = [15.16 3.59 31.05]';
com_offset_2 = [113.97 15.0 -19.02]';
com_offset_3 = [0.13 104.41 20.22]';
com_offset_4 = [0.15 53.53 -24.64]';
com_offset_5 = [0 1.2 -16.48]';

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
m1 = 1.39;
m2 = 1.318;
m3 = 0.821;
m4 = 0.769;
m5 = 0.687;
I1 = [0.0029525, 0.0060091, 0.0058821];
I2 = [0.0031145, 0.0005843, 0.0031631];
I3 = [0.00172767, 0.00041967, 0.0018468];
I4 = [0.0006764, 0.0010573, 0.0006610];
I5 = [0.0001934, 0.0001602, 0.0000689];

% Determine the mu matrices
M_1 = diag([m1 m1 m1 I1]);
M_2 = diag([m2 m2 m2 I2]);
M_3 = diag([m3 m3 m3 I3]);
M_4 = diag([m4 m4 m4 I4]);
M_5 = diag([m5 m5 m5 I5]);

% Derive the overall manipulator inertia matrix M
M = J1'*M_1*J1 + J2'*M_2*J2 + J3'*M_3*J3 + J4'*M_4*J4 + J5'*M_5*J5;
M = simplify(M);
disp(M);

%% Step 2 - Compute manipulator Coriolis matrix C
syms th1_dot th2_dot th3_dot th4_dot th5_dot 'real';
th = [th1 th2 th3 th4 th5];
th_dot = [th1_dot th2_dot th3_dot th4_dot th5_dot];

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
disp(N);

%% Step 4 - Compute the Equations of Motion

syms t TH1(t) TH2(t) TH3(t) TH4(t) TH5(t) 'real';

% Define new symbolics which let theta be a function of t
TH1_dot = diff(TH1, t);
TH2_dot = diff(TH2, t);
TH3_dot = diff(TH3, t);
TH4_dot = diff(TH4, t);
TH5_dot = diff(TH5, t);
TH1_ddot = diff(TH1_dot, t);
TH2_ddot = diff(TH2_dot, t);
TH3_ddot = diff(TH3_dot, t);
TH4_ddot = diff(TH4_dot, t);
TH5_ddot = diff(TH5_dot, t);

% Define some convenience vectors for the multiplication
TH = [TH1; TH2; TH3; TH4; TH5];
TH_dot = [TH1_dot; TH2_dot; TH3_dot; TH4_dot; TH5_dot];
TH_ddot = [TH1_ddot; TH2_ddot; TH3_ddot; TH4_ddot; TH5_ddot];

% Create the equations of motion
eom = M * TH_ddot + C * TH_dot + N;

% Substitute in for the parameters
params = struct;
params.th1 = TH1;
params.th2 = TH2;
params.th3 = TH3;
params.th4 = TH4;
params.th5 = TH5;
params.th1_dot = TH1_dot;
params.th2_dot = TH2_dot;
params.th3_dot = TH3_dot;
params.th4_dot = TH4_dot;
params.th5_dot = TH5_dot;

% Substitute parameters
eom = subs(eom, params);
eom = simplify(eom);

%% Controller - Analytic Solution

% Set the initial and desired parameters.
initial = [0 0 0 0 0]';
desired = [0, 0, 0, 0, 0.01]';

% Gains
kp = [50 50 50 50 50]';
kd = [10 10 10 10 10]';

% Analytic solution
fake_feedback = [0 0 0 0 0]';
error = fake_feedback - desired;
derror = [0 0 0 0 0]';

theta_feedback = struct;
theta_feedback.th1 = 0;
theta_feedback.th2 = 0;
theta_feedback.th3 = 0;
theta_feedback.th4 = 0;
theta_feedback.th5 = 0;

command_torques = M*(-diag(kd)*derror - diag(kp)*error) + N;
command_torques = subs(command_torques, theta_feedback);
disp(double(command_torques))

%% ODE45 Method

% Set the initial and desired parameters.
initial = [0 0 0 0 0 0 0 0 0 0]';
desired = [0, 0, 0, 0, pi/2, 0, 0, 0, 0, 0]';

% Create the controller. Here the controller only operates on angular error
% and not velocity error. Seems to work well enough.
kp = [50 50 50 50 50]';
kd = [10 10 10 10 10]';
error = desired(1:5) - TH;
controller = kp .* error + kd .* diff(error, t);

% Solve based on matlab reference.
% https://www.mathworks.com/help/symbolic/examples/solve-a-second-order-differential-equation-numerically.html?requestedDomain=true
time = [0 15];
[V, S] = odeToVectorField(eom == controller);
M = matlabFunction(V, 'vars', {'t', 'Y'});
sol = ode45(M, time, initial);

% Plot all the things.
subplot(2, 3, 1);
fplot(@(x)deval(sol, x, 3), time);
ref = refline(0, desired(1));
ref.Color = 'r';
title('TH1')

subplot(2, 3, 2);
fplot(@(x)deval(sol, x, 1), time);
ref = refline(0, desired(2));
ref.Color = 'r';
title('TH2');

subplot(2, 3, 3);
fplot(@(x)deval(sol, x, 5), time);
ref = refline(0, desired(3));
ref.Color = 'r';
title('TH3');

subplot(2, 3, 4);
fplot(@(x)deval(sol, x, 4), time);
ref = refline(0, desired(4));
ref.Color = 'r';
title('DTH1')

subplot(2, 3, 5);
fplot(@(x)deval(sol, x, 2), time);
ref = refline(0, desired(5));
ref.Color = 'r';
title('DTH2');

subplot(2, 3, 6);
fplot(@(x)deval(sol, x, 6), time);
ref = refline(0, desired(6));
ref.Color = 'r';
title('DTH3');