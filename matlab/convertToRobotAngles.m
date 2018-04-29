function data = convertToRobotAngles(angles)
%CONVERTTOROBOTANGLES Summary of this function goes here
%   Detailed explanation goes here
    zero_configuration = [-0.15, 1.1345, -2.5482, 1.7890, 2.9234];
    data = angles + zero_configuration;
end

