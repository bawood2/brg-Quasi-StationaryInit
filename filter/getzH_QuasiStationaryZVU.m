function [dz,H] = getzH_QuasiStationaryZVU(state)
%getzH_QuasiStationaryZVU : get measurement and measurement mapping
%matix for the Quasi-stationary zero velocity update implementation
%Inputs
%   state : data structure containing elements of the state
%
%Outputs
%   dz : measurement innovation 
%   H : measurement mapping matrix
%
%Reference
%   Equations from Chapter 13 : Inertial Navigation pg 416
%
%Log 
% 3/5/17 Brandon Wood : Initial Implementation
O = zeros(3,3);
I = eye(3);

dz = state.r0-state.r;
H = [ -I O O O O];

end