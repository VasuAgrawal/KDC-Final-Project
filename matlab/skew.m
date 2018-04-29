function s = skew(w)
%SKEW Summary of this function goes here
%   Detailed explanation goes here
s = [0 -w(3) w(2);
     w(3) 0 -w(1);
     -w(2) w(1) 0];
end

