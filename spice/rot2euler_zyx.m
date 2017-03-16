function [ Theta] = rot2euler_zyx( R )
%rot2euler_zyx : Converts coordinate tranformation matrix to a ZYX Euler
%angle sequence
%Inputs
%   R : Coordinate transformation matrix
%
%Outputs
%   Theta : ZYX Euler angle sequence (yaw,pitch,roll)
%
%Log
% 3/16/17 Brandon Wood : Initial Implementation

yaw = atan2(R(2,1),R(1,1));
pitch = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
roll = atan2(R(3,2),R(3,3));

Theta = [yaw;pitch;roll];


end

