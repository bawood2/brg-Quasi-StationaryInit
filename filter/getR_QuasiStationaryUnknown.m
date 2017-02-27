function [R] = getR_QuasiStationaryUnknown()
%Measurement noise covaraicne represents the variance of posisiont
%displacement due to vibration and disturbance


r11 = (10^-4)^2;
r22 = (10^-4)^2;
r33 = (10^-4)^2;
R = [r11, 0.0, 0.0;...
    0.0, r22, 0.0;...
    0.0, 0.0, r33];

end