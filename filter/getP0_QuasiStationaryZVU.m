function P = getP0_QuasiStationaryZVU()
% getP0_QuasiStationaryZVU : Compute initial state covariance matrix
% for quasi-stationary zero velocity update implementation
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
sigma_t = [0.0698, 0.0698,0.0698];  %4 degree uncertainty on all axis
sigma_ba = [0.01,0.01,0.01]; % initial guess
sigma_bg = [0.01,0.01,0.01]; % initial guess

P = diag( [ sigma_r.^2,sigma_v.^2,sigma_t.^2,sigma_ba.^2,sigma_bg.^2]);



end