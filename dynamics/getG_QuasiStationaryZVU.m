function G = getG_QuasiStationaryZVU()
%getG_QuasiStationaryZVU : get measurement mapping matrix
% matrix for QuasiStationary zero velocity update implemenetation
%Inputs
%
%Outputs
%   G : Matrix that maps process noise covariance onto state covaraince
%
%Log 
% 3/16/17 Brandon Wood : Initial Implementation

O = zeros(3,3);
o = zeros(3,1);
I = eye(3);

G = [ O O O O O;...
      O I O O O;...
      O O I O O;...
      O O O I O;...
      O O O O I];

end
