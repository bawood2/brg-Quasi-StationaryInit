function [ wx ] = skewmat( w )
%skewmat : Computes the skew symmetric matrix from a vector
%
%Inputs
%   w : 3 x 1 vector
%
%Outputs
%   wx : skew symmetric matrix
%
%Log 
% 3/5/17 Brandon Wood : Initial Implementation

wx = [ 0  -w(3) w(2);...
      w(3)  0  -w(1);...
     -w(2) w(1)   0 ];

end

