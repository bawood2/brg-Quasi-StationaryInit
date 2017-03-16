function Q = getQ_QuasiStationaryZVU(state,imu,params,PHI)
%getQ_QuasiStationaryZVU : get discrete process noise covaraince matrix 
%for the Quasi-stationary zero velocity update implementation
%Inputs
%
%Outputs
%   Q : Discrete time process noise covariance
%
%Reference
%   Discrete-time Solutions to the Continuous-time Differential Lyapunov
%   Equation With Applications to Kalman Filtering (P.Axelsson &
%   F.Grustafsson)
%Log 
% 3/5/17 Brandon Wood : Initial Implementation
% 3/16/17 Brandon Wood : Updated Model

n = state.n;
dt = imu.dt;
F = params.G*params.Q*params.G';
Q = PHI * (eye(n) + F*dt + 0.5*F*F*dt^2 +1.0/6.0*F*F*F*dt^3)';

end