% rosinit('localhost'); % On localhost
clc;

pub = rospublisher('/angle_setpoints');
msg = rosmessage('std_msgs/Float32MultiArray');
msg.Data = convertToRobotAngles([pi, 0, 0, pi/2, 0]);
send(pub, msg);

torques = [];


torque_sub = rossubscriber('/torque_feedback');
torque_feedback = receive(torque_sub, 1);
while 1

    
end




feedback_sub = rossubscriber('/angle_feedback');
feedback = receive(feedback_sub, 1);
showdetails(feedback);





feedback = receive(feedback_sub, 1);
showdetails(feedback);