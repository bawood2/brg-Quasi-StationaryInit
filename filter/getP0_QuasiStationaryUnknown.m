function P = getP0_QuasiStationaryUnknown()
% getP0_QuasiStationaryUnknown : Compute initial state covariance matrix
% for quasi-stationary unknown heading implementation
%
%Outputs
%   P : state covariance matrix
%
%Reference
%   Equations from Groves Chapter  : pg 
%
%Log 
% 3/5/17 Brandon Wood : Initial Implementation

sigma_r = [0.0162,0.0144,2]; %Taken as the difference between champaign and campus lat/long/elevation
sigma_v = [10^-7,10^-7,10^-7]; % initial guess
sigma_t = [pi/2, 0.0698,0.0698];  %4 degree uncertainty for pitch and roll
sigma_sinSi = 1; % initial guess
sigma_cosSi = 1; % initial guess
sigma_ba = [0.001,0.001,0.001]; % initial guess
sigma_bg = [0.001,0.001,0.001]; % initial guess

P = diag( [ sigma_r.^2,sigma_v.^2,sigma_t.^2,sigma_sinSi.^2,sigma_cosSi.^2,sigma_ba.^2,sigma_bg.^2]);



end