function state = quasiStationaryAlignZVU(state,params,imu)

%Propogate state with IMU measurements
state = imuStatePropogationLocalFrame(state,params,imu);

%Error and uncertainty propogation
F = getF_QuasiStationaryZVU(state,params,imu); % Get State Jacobian
PHI = getPHI_QuasiStationaryZVU(state,imu,F); % Get state transition matrix
Q = getQ_QuasiStationaryZVU(state,imu,params,PHI);
%state.dx = PHI*state.dx; % Since states are zeroed, no need to propogate
Pk = PHI*state.P*PHI' + Q; % Propogate uncertainty

if params.loopCount>5000
    keyboard;
end

%Compute Kalman Gain
[dz,H] = getzH_QuasiStationaryZVU(state); % measurement and mapping
Gamma = H*Pk*H' + params.R; % Innovations
K = Pk*H'/Gamma;


%Update state and covariance
I = eye(state.n);
state.P = (I - K * H )*Pk*(I - K * H)' + K * params.R * K'; % Josheph form 
state.dx = state.dx + K*dz;

%Output
state.r = state.r - state.dx(1:3);
state.v = state.v - state.dx(4:6);
state.t = state.t - state.dx(7:9);
%state.T =  (eye(3) - skewmat(state.t))*state.T; % small angle approx
state.T = R_ZYX(state.dx(7),state.dx(8),state.dx(9))'*state.T;
state.T = reOrthoNorm(state.T);
state.b_a = state.dx(10:12);
state.b_g = state.dx(13:15);

%Zero Kalman Filter Estimate of position, velocity, attitude ( see pg 366)
state.dx(1:end) = zeros(state.n,1);

end
