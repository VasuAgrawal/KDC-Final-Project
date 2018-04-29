function [t] = twistM(w,q)
%TWIST Summary of this function goes here
%   Detailed explanation goes here

v = cross(-w, q);
t = [skew(w), v;
     0 0 0 0];
end

