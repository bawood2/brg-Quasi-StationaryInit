function R = R_ZYX(t1,t2,t3)
%R_ZYZ
%Inputs
%   t1 : yaw
%   t2 : pitch
%   t3 : roll
%
%Ouputs
%   R : ZYX (yaw,pitch,roll) Euler Sequence coordinate transformation

% Define a Rotation Matrix for ZYX Euler sequence
Rz = [ cos(t1) -sin(t1) 0 ; sin(t1) cos(t1) 0 ; 0 0 1];
Ry = [ cos(t2) 0 sin(t2) ; 0 1 0 ; -sin(t2) 0 cos(t2)];
Rx = [ 1 0 0 ; 0 cos(t3) -sin(t3) ; 0 sin(t3) cos(t3)];

R = Rz*Ry*Rx;   % - compute orientation from Euler Angles (state.theta1, state.theta2, state.theta3)

end