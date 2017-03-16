function [theta,phi] = levelingProcess(f)
%levelingProcess : initializes the pitch and roll angles of a stationary
%INS
%Inputs
%   f : time average specific force of body wrt inertial in inertial
%
%Outputs
%   theta : elevation angle (pitch) of body frame wrt to local navigation frame
%   phi : bank angle (roll) of body fram wrt to local navigation frame
%
%References
%   Equations from Groves Chapter 5 : Inertial Navigation pg 148
%
%Log 
% 3/5/17 Brandon Wood : Initial Implementation


%Elevation (pitch)
theta = atan( -f(1) / sqrt( f(2)^2 + f(3)^2 ) );

%Bank (roll)
phi = atan2( -f(2), -f(3) );




end