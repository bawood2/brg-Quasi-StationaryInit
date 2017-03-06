function F = getF_QuasiStationaryUnknown(state,params,imu)
%getF_QuasiStationaryUnknown : get measurement and measurement mapping
%matix for the Quasi-stationary unknown heading implementation
%Inputs
%   state : data structure containing elements of the state
%   params : data structure containing additional parameters
%   imu: data structure containing imu data
%Outputs
%   F : system transition matrix.  Jocobian  d xdot / d x
%
%Reference
%   Equations from Chapter 13 : INS Alignment pg 414
%
%Log 
% 3/5/17 Brandon Wood : Initial Implementation
O = zeros(3,3);
o = zeros(3,1);
I = eye(3);

T = state.T; % Coordinate transformation matrix
f = imu.f - state.b_a; % specific force

wie = params.omegaIE* [ state.cosSi * params.cosL ; -state.sinSi*params.cosL ; params.sinL ] ;

%Compute position Jocobian
drdot_dx = [ O I O o o O O];

%Compute velocity Jocobian
dvdot_dsi = -skewmat(T*f);

dvdot_dx = [ O O dvdot_dsi o o T O];

%Compute attitude Jacobian
dsidot_dsi = -skewmat(wie);
dsidot_dsin = [ 0 ;params.omegaIE*params.cosL ; 0 ];
dsidot_dcos = [ -params.omegaIE*params.cosL; 0 ; 0];

dsidot_dx = [ O O dsidot_dsi dsidot_dsin dsidot_dcos O T];

%Compute wander aziumuth angle Jacobains
dsin_dx = zeros(1,state.n);
dcos_dx = zeros(1,state.n);

%Compute accelerometer bias Jacobain
dba_dx = [O O O o o O O];

%Compute gyro bias Jacobian
dbg_dx = [O O O o o O O];

%Compute F
F = [ drdot_dx ; dvdot_dx ; dsidot_dx ; dsin_dx ; dcos_dx ; dba_dx ; dbg_dx ];




end