function state = quasiStationaryAlignZVU(state,params,imu)

%Propogate state with IMU measurements
state = imuStatePropogationZVU(state,params,imu);

%Error and uncertainty propogation
F = getF_QuasiStationaryZVU(state,params,imu); % Get State Jacobian
PHI = getPHI_QuasiStationaryZVU(state,F); % Get state transition matrix
%state.dx = PHI*state.dx; % Since states are zeroed, no need to propogate
Pk = PHI*state.P*PHI' + params.Q*params.dt; % Propogate uncertainty

%Compute Kalman Gain
[dz,H] = getzH_QuasiStationaryZVU(state); % measurement and mapping
Gamma = H*P*H' + params.R; % Innovations
K = Pk*H'/Gamma;


%Update state and covariance
I = eye(state.n);
state.P = (I - K * H )*Pk*(I - K * H)' + K * params.R * K'; % Josheph form 
state.dx = state.dx + K*dz;

%Output
state.r = state.r - state.dx(1:3);
state.v = state.v - state.dx(4:6);
state.t = state.t - state.dx(7:9);
state.T =  (eye(3) - skewmat(state.t))*state.T; % small angle approx
state.b_a = state.b_a - state.dx(12:14);
state.b_g = state.b_g - state.dx(15:17);

%Zero Kalman Filter Estimate of position, velocity, attitude ( see pg 366)
state.dx(1:9) = zeros(9,1);

end
