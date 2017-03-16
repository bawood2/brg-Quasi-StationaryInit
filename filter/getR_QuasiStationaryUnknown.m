function [R] = getR_QuasiStationaryUnknown()
%getR_QuasiStationaryUnknown : Compute measurement noise covariance for
%quasistationary unknown heading implementation
%
%Outputs
%   R : measurement noise covariance
%
%Notes
%   Measurement noise covariance represents the variance of position
%   displacement due to vibration and disturbance
%
%Reference
%   Equations from Groves Chapter 13 : pg 415 
%
%Log 
% 3/5/17 Brandon Wood : Initial Implementation


r11 = (10^-6)^2;
r22 = (10^-6)^2;
r33 = (10^-6)^2;
R = [r11, 0.0, 0.0;...
    0.0, r22, 0.0;...
    0.0, 0.0, r33];

end