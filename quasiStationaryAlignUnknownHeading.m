function state = quasiStationaryAlignUnknownHeading(state,params,imu)

%Propogate state with IMU measurements
state = imuStatePropogationUnknown(state,params,imu);

%Error and uncertainty propogation
F = getF_QuasiStationaryUnknown(state,params,imu); % Get State Jacobian
PHI = getPHI_QuasiStationaryUnknown(state,F); % Get state transition matrix
%state.dx = PHI*state.dx; % Since states are zeroed, no need to propogate
Pk = PHI*state.P*PHI' + params.Q*params.dt; % Propogate uncertainty

%Compute Kalman Gain
[dz,H] = getzH_QuasiStationaryUnknown(state); % measurement and mapping
Gamma = H*P*H' + params.R; % Innovations
K = Pk*H'/Gamma;


%Update state and covariance
I = eye(state.n);
state.P = (I - K * H )*Pk*(I - K * H)' + K * params.R * K'; % Josheph form 
state.dx = state.dx + K*dz;

%Output
state.dr = state.dr - state.dx(1:3);
state.v = state.v - state.dx(4:6);
state.t = state.t - state.dx(7:9);
state.T = R_XYZ(state.dx(7),state.dx(8),state.dx(9))'*state.T; % Errors may not yet be <2deg
state.state.sinSi = state.state.sinSi - state.dx(10);
state.state.cosSi = state.state.cosSi - state.dx(11);
state.b_a = state.b_a - state.dx(12:14);
state.b_g = state.b_g - state.dx(15:17);

%Zero Kalman Filter Estimate of position, velocity, attitude ( see pg 366)
state.dx(1:9) = zeros(9,1);





end
