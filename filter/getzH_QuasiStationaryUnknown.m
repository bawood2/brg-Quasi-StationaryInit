function [dz,H] = getzH_QuasiStationaryUnknown(state)
%getzH_QuasiStationaryUnknown : get measurement and measurement mapping
%matix for the Quasi-stationary unknown heading implementation
%Inputs
%   state : data structure containing elements of the state
%
%Outputs
%   dz : measurement innofvation dz = -Delta(r)
%   H : measurement mapping matrix
%
%Reference
%   Equations from Chapter 13 : Inertial Navigation pg 415
%
%Log 
% 3/5/17 Brandon Wood : Initial Implementation

O = zeros(3,3);
o = zeros(3,1);
I = eye(3);

dz = -state.dr;
H = [ -I O O o o O O];

end