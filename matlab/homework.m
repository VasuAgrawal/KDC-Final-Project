clear;
close all;
clc;

% Define symbolics
syms th1 th2 'real';
syms l0 l1 'real';
syms r0 r1 'real';

% Define the twists
xi_1 = [0 0 0 0 0 1]';
xi_2 = [0 -l0 0 -1 0 0]';

% Define forward kinematics. I suppose this could have been computed too.
gsl1_0 = [eye(3) [0 0 r0]';
          [0 0 0] 1];
gsl2_0 = [eye(3) [0 r1 l0]';
          [0 0 0] 1];

%% 1.1
      
% Derive Jacobians using the formula.
J1 = [inv(adj(expm(wedge(xi_1)*th1) * gsl1_0)) * xi_1, zeros(6, 1)];
J2 = [inv(adj(expm(wedge(xi_1)*th1) * expm(wedge(xi_2)*th2) * gsl2_0)) * xi_1, ...
      inv(adj(expm(wedge(xi_2)*th2) * gsl2_0)) * xi_2];
J1 = simplify(J1);
J2 = simplify(J2);

% Define parameters for the inertia matrix.
syms m1 m2 'real';
syms Ix1 Iy1 Iz1 Ix2 Iy2 Iz2 'real';

% Create the inertia matrices.
M_1 = diag([m1 m1 m1 Ix1 Iy1 Iz1]);
M_2 = diag([m2 m2 m2 Ix2 Iy2 Iz2]);

% Derive the overall inertia matrix.
M = J1'*M_1*J1 + J2'*M_2*J2;
M = simplify(M);
fprintf('Overall Mass Matrix \n');
disp(M); % Show it.

%% 1.2

% Define theta and other symbolics.
syms th1_dot th2_dot 'real';
syms th1_ddot th2_ddot 'real';
th = [th1 th2];
th_dot = [th1_dot th2_dot];
th_ddot = [th1_ddot th2_ddot];

n = numel(th);
C = sym(zeros(n));

% Add elements to the C matrix per the formula. Make sure to simplify.
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

% Simplify again!
C = simplify(C);
fprintf('Overall Coriolis Matrix \n');
disp(C);

%% 1.3

% Come up with the forward kinematics to each frame.
gsl1_th = expm(wedge(xi_1) * th1) * gsl1_0;
gsl2_th = expm(wedge(xi_1) * th1) * expm(wedge(xi_2) * th2) * gsl2_0;

% Figure out the positions of each frame in world coordinates.
p = [0 0 0 1]';
L1 = simplify(gsl1_th * p);
L2 = simplify(gsl2_th * p);

% Use that to extract the Z component.
h1 = L1(3);
h2 = L2(3);

% Define V in terms of gravity.
syms g
V = m1*g*h1 + m2*g*h2;

% Solve for N.
N = [diff(V, th1);
     diff(V, th2)];

N = simplify(N);
fprintf('Overall Normal Matrix \n');
disp(N);

%% Rearranging the equations

syms T1 T2 'real';
T = [T1 T2]; % Torques

EoM = M * th_ddot' + C * th_dot' + N;
eqns = EoM == T';

% Once we've generated the full equations of motion, substitute in for the
% various values th_dot, th_ddot, th.
params = struct;
params.th1 = 1;
params.th1_dot = 1;
params.th1_ddot = 1;
params.th2 = 1;
params.th2_dot = 1;
params.th2_ddot = 1;
params.g = 9.81;
params.m1 = 1;
params.m2 = 1;

sub_eqns = subs(eqns, params);
sub_eqns = simplify(sub_eqns);

syms u1 'real';
sub_eqns = subs(sub_eqns, r1^2, u1);

% %% 1.4 TODO
% 
% syms t TH1(t) TH2(t) TH3(t) 'real';
% 
% % Define new symbolics which let theta be a function of t.
% TH1_dot = diff(TH1, t);
% TH2_dot = diff(TH2, t);
% TH3_dot = diff(TH3, t);
% TH1_dot_dot = diff(TH1_dot, t);
% TH2_dot_dot = diff(TH2_dot, t);
% TH3_dot_dot = diff(TH3_dot, t);
% 
% % Define some convenience vectors for the multiplication.
% TH = [TH1; TH2; TH3];
% TH_dot = [TH1_dot; TH2_dot; TH3_dot];
% TH_dot_dot = [TH1_dot_dot; TH2_dot_dot; TH3_dot_dot];
% 
% % Create the equations of motion.
% eom = M * TH_dot_dot + C * TH_dot + N;
% 
% % Substitute in for the parameters.
% params = struct;
% params.l0 = 1;
% params.l1 = 1;
% params.l2 = 1;
% params.r0 = 1;
% params.r1 = 1;
% params.r2 = 1;
% params.m1 = 1;
% params.m2 = 1;
% params.m3 = 1;
% params.Ix1 = 1;
% params.Iy1 = 1;
% params.Iz1 = 1;
% params.Ix2 = 1;
% params.Iy2 = 1;
% params.Iz2 = 1;
% params.Ix3 = 1;
% params.Iy3 = 1;
% params.Iz3 = 1;
% params.g = 9.81;
% params.th1 = TH1;
% params.th2 = TH2;
% params.th3 = TH3;
% params.th1_dot = TH1_dot;
% params.th2_dot = TH2_dot;
% params.th3_dot = TH3_dot;
% 
% % And actually substitute.
% eom = subs(eom, params);
% eom = simplify(eom);
% 
% % Set the initial and desired parameters.
% initial = [0 0 0 0 0 0]';
% desired = [0, -pi/2, 0, 0, 0, 0]';
% 
% % Create the controller. Here the controller only operates on angular error
% % and not velocity error. Seems to work well enough.
% kp = [50 50 50]';
% kd = [10 10 10]';
% error = desired(1:3) - TH;
% controller = kp .* error + kd .* diff(error, t);
% 
% % Solve based on matlab reference.
% % https://www.mathworks.com/help/symbolic/examples/solve-a-second-order-differential-equation-numerically.html?requestedDomain=true
% time = [0 15];
% [V, S] = odeToVectorField(eom == controller);
% M = matlabFunction(V, 'vars', {'t', 'Y'});
% sol = ode45(M, time, initial);
% 
% % Plot all the things.
% subplot(2, 3, 1);
% fplot(@(x)deval(sol, x, 3), time);
% ref = refline(0, desired(1));
% ref.Color = 'r';
% title('TH1')
% 
% subplot(2, 3, 2);
% fplot(@(x)deval(sol, x, 1), time);
% ref = refline(0, desired(2));
% ref.Color = 'r';
% title('TH2');
% 
% subplot(2, 3, 3);
% fplot(@(x)deval(sol, x, 5), time);
% ref = refline(0, desired(3));
% ref.Color = 'r';
% title('TH3');
% 
% subplot(2, 3, 4);
% fplot(@(x)deval(sol, x, 4), time);
% ref = refline(0, desired(4));
% ref.Color = 'r';
% title('DTH1')
% 
% subplot(2, 3, 5);
% fplot(@(x)deval(sol, x, 2), time);
% ref = refline(0, desired(5));
% ref.Color = 'r';
% title('DTH2');
% 
% subplot(2, 3, 6);
% fplot(@(x)deval(sol, x, 6), time);
% ref = refline(0, desired(6));
% ref.Color = 'r';
% title('DTH3');