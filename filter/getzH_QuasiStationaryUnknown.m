function [dz,H] = getzH_QuasiStationaryUnknown(state)

O = zeros(3,3);
o = zeros(3,1);
I = eye(3);

dz = -state.dr;
H = [ O O -I o o O O];

end