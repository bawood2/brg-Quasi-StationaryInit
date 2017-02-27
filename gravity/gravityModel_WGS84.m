function params = gravityModel_WGS84 (L,h,params)
%gravityModel_WGS84 : WGS 84 gravity model
%Inputs
%   L : latitude
%   h : altitude
%
%Outputs
%   g : Down component of gravity with respect to the body frame resolved
%   in the local navigation frame
%
%Notes
%   Chapter 2 Coordinate Frames, Kinemeatics, and the Earth, pgs and 38, 45, 47, 48


%WGS 84 Earth Ellipsoid Model
R0 = 6378137.0; % Earths equitorial radius [m]
RP = 6356752.3142; % Earths polar radius  [m]
f = 1 / 298.257223563; % Earths flattening
e = 0.0818191908425; % Earths eccentricity

%WGS 84 Earth physical characteristics
omega = 7.292115 * 10 ^ -5; % Earths angular rate [rad/s]
mu = 3.986004418*10^14; % Earths gravitational constant [m^3 / s^2]


%Constants
alpha = 9.7803253359;
beta = 0.001931853;

sinL2 = sin(L)^2;
g0 = alpha * (1.0 + beta*sinL2) / sqrt( 1.0 - e^2*sinL2 ) ;

g = g0 * ( 1.0 - 2.0 / R0 * ( 1.0 + f + omega^2*R0^2*RP/mu )*h + 3.0 / R0^2 * h^2 );


%Record parameters
params.g = g;
params.R0 = R0; % Earths equitorial radius [m]
params.RP = RP; % Earths polar radius  [m]
params.f = f; % Earths flattening
params.e = e; % Earths eccentricity

%WGS 84 Earth physical characteristics
params.omegaE= omega; % Earths angular rate [rad/s]
params.mu = mu; % Earths gravitational constant [m^3 / s^2]
