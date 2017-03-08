function state = quasiStationaryAlignUnknownHeading(state,params,imu)

%Propogate state with IMU measurements
state = imuStatePropogationUnknown(state,params,imu);


%Error and uncertainty propogation
F = getF_QuasiStationaryUnknown(state,params,imu); % Get State Jacobian
PHI = getPHI_QuasiStationaryUnknown(state,imu,F); % Get state transition matrix
%state.dx = PHI*state.dx; % Since states are zeroed, no need to propogate
Pk = PHI*state.P*PHI' + params.Q*imu.dt; % Propogate uncertainty

if params.loopCount>100
    keyboard;
end

%Compute Kalman Gain
[dz,H] = getzH_QuasiStationaryUnknown(state); % measurement and mapping
Gamma = H*Pk*H' + params.R; % Innovations
K = Pk*H'/Gamma;


%Update state and covariance
I = eye(state.n);
state.P = (I - K * H )*Pk*(I - K * H)' + K * params.R * K'; % Josheph form 
state.dx = state.dx + K*dz;

%Output
state.dr = state.dr - state.dx(1:3);
state.v = state.v - state.dx(4:6);
state.t = state.t - state.dx(7:9);
state.sinSi = state.sinSi - state.dx(10);
state.cosSi = state.cosSi - state.dx(11);
state.T = R_ZYX(state.dx(7),state.dx(8),state.dx(9))'*state.T; % Errors may not yet be <2deg
%state.T = T_ZYX_Unknown(state.sinSi,state.cosSi,state.dx(8),state.dx(9))'*state.T; % Errors may not yet be <2deg
%state.T = R_ZYX(state.t(1),state.t(2),state.t(3));
%dT = [state.dx(11), state.dx(10), 0.0; -state.dx(10), state.dx(11), 0.0; 0.0,0.0,1.0]*(eye(3) + skewmat([state.dx(9),state.dx(8),0]));
%state.T = dT*state.T;
state.T = reOrthoNorm(state.T);
state.b_a = state.b_a - state.dx(12:14);
state.b_g = state.b_g - state.dx(15:17);

%Zero Kalman Filter Estimate of position, velocity, attitude ( see pg 366)
state.dx(1:end) = zeros(state.n,1);


end

function T = T_ZYX_Unknown(sinSi,cosSi,theta,phi)
% Define a Rotation Matrix for ZYX Euler sequence
cosTheta = cos(theta);
sinTheta = sin(theta);
cosPhi = cos(phi);
sinPhi = sin(phi);
Rz = [ cosSi -sinSi 0 ; sinSi cosSi 0 ; 0 0 1];  %yaw
Ry = [ cosTheta 0 sinTheta ; 0 1 0 ; -sinTheta 0 cosTheta]; %pitch
Rx = [ 1 0 0 ; 0 cosPhi -sinPhi ; 0 sinPhi cosPhi]; %roll

T = Rz*Ry*Rx;   % - compute orientation from Euler Angles (state.theta1, state.theta2, state.theta3)
end
