function [tool_pos] = forwardKinematics(th)
% Base frame: Center of the arm, y pointing forward.
t = [0, -33, 608]'; % Tool frame is base of the gripper
R = eye(3);
gst0 = [R, t;
        0 0 0 1];

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

% Twist matrices
t1 = twistM(w1, q1);
t2 = twistM(w2, q2);
t3 = twistM(w3, q3);
t4 = twistM(w4, q4);
t5 = twistM(w5, q5);

gripper_tip = [0 0 47 1]';
gst = expm(t1 * th(1)) * expm(t2 * th(2)) * ...
      expm(t3 * th(3)) * expm(t4 * th(4)) * ...
      expm(t5 * th(5)) * gst0;
tool_pos = gst * gripper_tip;
tool_pos = tool_pos(1:3);
end

