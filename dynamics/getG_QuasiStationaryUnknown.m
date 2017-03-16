function G = getG_QuasiStationaryUnknown(state)
%getG_QuasiStationaryUnknown : get measurement mapping matrix
% matrix for QuasiStationary Unknown heading implemenetation
%Inputs
%   state : data structure containing elements of the state
%
%Outputs
%   G : Matrix that maps process noise covariance onto state covaraince
%
%Log 
% 3/16/17 Brandon Wood : Initial Implementation

O = zeros(3,3);
o = zeros(3,1);
I = eye(3);

G = [ O O O o o O O;...
      O I O o o O O;...
      O O I o o O O;...
      zeros(2,state.n);...
      O O O o o I O;...
      O O O o o O I];

end
