function si = gyrocompassingProcess(w,theta,phi)
%gyrocompassingProcess : initializes the yaw angle of a stationary INS
%
%Inputs
%   w : time average angular rate from inertial frame to body frame wrt to body frame
%   theta : elevation angle (pitch) of body frame wrt to local navigation frame
%   phi : bank angle (roll) of body frame wrt to local navigation frame
%
%Outputs
%   si : heading azimuth (yaw) of body frame wrt to local navigation frame
%
%Notes
%   gyrocompassing is likely not possible for low quality / low cost IMU
%
%Reference
%   Equations from Chapter 5 : Inertial Navigation pg 149
%
%Log 
% 3/5/17 Brandon Wood : Initial Implementation

%Reduce computation
cosPhi = cos(phi);
sinPhi = sin(phi);
cosTheta = cos(theta);
sinTheta = sin(theta);

%Angular rate by component
wx = w(1);
wy = w(2);
wz = w(3);

sinSi = -wy*cosPhi + wz*sinPhi;
cosSi = wx*cosTheta + wy*sinPhi*sinTheta + wz*cosPhi*sinTheta;

%Heading (Yaw)
si = atan2(sinSi,cosSi);


end