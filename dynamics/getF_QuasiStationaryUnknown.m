function F = getF_QuasiStationaryUnknown(state,params,imu)

O = zeros(3,3);
o = zeros(3,1);
I = eye(3);

T = state.T; % Coordinate transformation matrix
f = imu.f - state.b_a; % specific force

wie = params.omegaE* [ state.cosSi * params.cosL ; -state.sinSi*params.cosL ; params.sinL ] ;

%Compute position Jocobian
drdot_dx = [ O I O o o O O];

%Compute velocity Jocobian
dvdot_dsi = -skewmat(T*f);

dvdot_dx = [ O O dvdot_dsi o o T O];

%Compute attitude Jacobian
dsidot_dsi = -skewmat(omega);
dsidot_dsin = [ 0 ;wie*params.cosL ; 0 ];
dsidot_dcos = [ -wie*params.cosL; 0 ; 0];

dsidot_dx = [ O O dsidot_dsi dsidot_dsin dsidot_dcos O T];

%Compute wander aziumuth angle Jacobains
dsin_dx = zeros(state.n,1);
dcos_dx = zeros(state.n,1);

%Compute accelerometer bias Jacobain
dba_dx = [O O O o o O O];

%Compute gyro bias Jacobian
dbg_dx = [O O O o o O O];

%Compute F
F = [ drdot_dx ; dvdot_dx ; dsidot_dx ; dsin_dx ; dcos_dx ; dba_dx ; dbg_dx ];




end