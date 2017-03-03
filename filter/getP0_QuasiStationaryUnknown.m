function P = getP0_QuasiStationaryUnknown()

sigma_r = [0.0162,0.0144,2]; %Taken as the difference between champaign and campus lat/long/elevation
sigma_v = [10^-6,10^-6,10^-6]; % initial guess
sigma_t = [pi/2, 0.0698,0.0698];  %4 degree uncertainty
sigma_sinSi = 1; % initial guess
sigma_cosSi = 1; % initial guess
sigma_ba = [0.1,0.1,0.1]; % initial guess
sigma_bg = [0.1,0.1,0.1]; % initial guess

P = diag( [ sigma_r^2,sigma_v^2,sigma_t^2,sigma_sinSi^2,sigma_cosSi^2,sigma_ba^2,sigma_bg^2]);



end