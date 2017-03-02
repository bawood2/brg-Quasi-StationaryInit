function R = R_ZYX(t1,t2,t3)
%R_ZYZ
%Inputs
%   t1 : yaw
%   t2 : pitch
%   t3 : roll
%
%Ouputs
%   R : ZYX (yaw,pitch,roll) Euler Sequence coordinate transformation
cosSi = cos(t1);
sinSi = sin(t1);
cosTheta = cos(t2);
sinTheta = sin(t2);
cosPhi = cos(t3);
sinPhi = sin(t3);


% Define a Rotation Matrix for ZYX Euler sequence
Rz = [ cosSi -sinSi 0 ; sinSi cosSi 0 ; 0 0 1];
Ry = [ cosTheta 0 sinTheta ; 0 1 0 ; -sinTheta 0 cosTheta];
Rx = [ 1 0 0 ; 0 cosPhi -sinPhi ; 0 sinPhi cosPhi];

R = Rz*Ry*Rx;   % - compute orientation from Euler Angles (state.theta1, state.theta2, state.theta3)

end