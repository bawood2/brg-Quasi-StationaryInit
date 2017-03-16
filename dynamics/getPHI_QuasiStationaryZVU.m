function PHI = getPHI_QuasiStationaryZVU(state,F)
%getPHI_QuasiStationaryZVU : get state transition matrix for the
%Quasi-stationary zero velocity update implementation
%Inputs
%   state : data structure containing elements of the state
%   imu: data structure containing imu data
%   F :system transition matrix.  Jocobian  d xdot / d x
%Outputs
%   PHI : state transition matrix.  Transistions the state from tkm1 to tk
%   in discrete time over the fixed time interval dt = (tkm1-tk).
%
%Reference
%   Groves Chapter 3 : Kalman Filter pg 68
%
%Log 
% 3/5/17 Brandon Wood : Initial Implementation
n = state.n;
dt = imu.dt;
PHI = (eye(n) + F*dt + 0.5*F*F*dt^2 +1.0/6.0*F*F*F*dt^3);


end