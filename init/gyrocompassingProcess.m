function si = gyrocompassingProcess(w,theta,phi)
%gyrocompassingProcess
%Inputs
%   w : angular rate from inertial frame to body frame wrt to body frame
%   theta : elevation angle (pitch) of body frame wrt to local navigation frame
%   phi : bank angle (roll) of body frame wrt to local navigation frame
%
%Outputs
%   si : heading azimuth (yaw) of body frame wrt to local navigation frame
%
%Notes
%   Equations from Chapter 5 : Inertial Navigation pg 149

cosPhi = cos(phi);
sinPhi = sin(phi);
cosTheta = cos(theta);
sinTheta = sin(theta);

wx = w(1);
wy = w(2);
wz = w(3);

sinSi = -wy*cosPhi + wz*sinPhi;
cosSi = wx*cosTheta + wy*sinPhi*sinTheta + wz*cosPhi*sinTheta;
si = atan2(sinSi,cosSi);


end