function data = convertToRobotAngles(angles)
%CONVERTTOROBOTANGLES Summary of this function goes here
%   Detailed explanation goes here
    global zero_configuration;
    data = angles + zero_configuration;
end

