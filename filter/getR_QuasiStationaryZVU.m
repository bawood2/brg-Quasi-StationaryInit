function [R] = getR_QuasiStationaryZVU()
%Measurement noise covaraicne represents the variance of posisiont
%displacement due to vibration and disturbance

r11 = (10^-6)^2;
r22 = (10^-6)^2;
r33 = (10^-6)^2;
R = [r11, 0.0, 0.0;...
    0.0, r22, 0.0;...
    0.0, 0.0, r33];

end