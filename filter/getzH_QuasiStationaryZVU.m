function [dz,H] = getzH_QuasiStationaryZVU(state)

O = zeros(3,3);
I = eye(3);

dz = state.r0-state.r;
H = [ O O -I O O];

end