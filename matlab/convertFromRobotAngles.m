function data = convertFromRobotAngles(angles)
%CONVERTFROMROBOTANGLES Summary of this function goes here
%   Detailed explanation goes here
    global zero_configuration;
    data = angles - zero_configuration;
end

