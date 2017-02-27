function [theta,phi] = levelingProcess(f)
%levelingProcess
%Inputs
%   f : specific force of body wrt inertial in inertial
%
%Outputs
%   theta : elevation angle (pitch) of body frame wrt to local navigation frame
%   phi : bank angle (roll) of body fram wrt to local navigation frame
%
%Notes
%   Equations from Chapter 5 : Inertial Navigation pg 148


%Elevation
theta = atan( -f(1) / sqrt( f(2)^2 + f(3)^2 ) );

%Bank
phi = atan2( -f(2), -f(3) );




end