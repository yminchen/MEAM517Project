function [leg_angle] = approx_leg_angle(phi,alpha,beta)
%APPROX_LEG_ANGLE Summary of this function goes here
%   Detailed explanation goes here

    leg_angle = -(phi+alpha+beta/2);
end

