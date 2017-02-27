function [ wx ] = skewmat( w )
%Creates a skew symmetric matrix given a 1x3 vector

wx = [ 0  -w(3) w(2);...
      w(3)  0  -w(1);...
     -w(2) w(1)   0 ];

end

