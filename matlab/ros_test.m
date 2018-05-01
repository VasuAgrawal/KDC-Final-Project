clc;

global zero_configuration;
zero_configuration = [-0.15, 1.1345, -2.5482, 1.7890, 2.9234];

pub = rospublisher('/angle_setpoints', 'IsLatching', false);
all_sub = rossubscriber('/combined_feedback');

% Send it back to the "reset" position
msg = rosmessage('std_msgs/Float32MultiArray');
angle_setpoint = [pi, 0, 0, 0, 0];
msg.Data = convertToRobotAngles(angle_setpoint);
send(pub, msg);

fprintf("Press Enter once the robot is pointing straight up.\n");
pause;

% Now command it to go somewhere else and start recording
angle_setpoint = [3*pi/4, pi/4, 0, pi/4, 0];
msg.Data = convertToRobotAngles(angle_setpoint);
send(pub, msg);


% Start recording
angles = [];
velocities = [];
torques = [];
currents = [];


while 1
    feedback = receive(all_sub, 1);
    data = feedback.Data;
    
    % angle, velocity, torque, current
    angle_feedback = convertFromRobotAngles(data(1:4:end)');
    velocity_feedback = data(2:4:end)';
    torque_feedback = data(3:4:end)';
    current_feedback = data(4:4:end)';
    
    angles = [angles; angle_feedback];
    velocities = [velocities; velocity_feedback];
    torques = [torques; torque_feedback];
    currents = [currents; current_feedback];
    
    diff = angle_setpoint - angle_feedback;
    if norm(diff) < 0.001
        fprintf("Got close enough, not recording any more.\n");
        break
    end
end

save("weighted_run4.mat", "angles", "velocities", "torques", "currents");




% 
% torque_sub = rossubscriber('/torque_feedback');
% 
% 
% torques_withWeight = [];
% 
% torque_sub = 

% %while 1
% for i=1:1000
%     torque_feedback = receive(torque_sub, 1);
%     data = torque_feedback.Data;
%     
%     torques_withWeight = [torques_withWeight; data'];
%     
% %     plot(torques(:, 1), torques(:, 2), torques(:, 3);
%     plot(torques_withWeight(:, 4));
%     drawnow;
%     
%     pause(0.05);
% end
