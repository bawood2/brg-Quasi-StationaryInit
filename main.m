%quasiStationaryAlignUnknownHeading test script

%2/18/17 Notes
% Check coordinate transformation initialization. May need to do a
% transpose

% Initialize Data.  These should all be inputs
specificForce = 1; % Load IMU specific force measurements
angularRate = 1; % Load IMU angular rate measurements
dt = 1;
L = 1;
lambda = 1;
h = 1;

%In system, IMU measurements are being taken.  Prior to alignment, need to
%initialize orientation.  This is done by a leveling process. Prior to
%this, a time average of IMU is taken.

counter = 1;
countermax = 20;
fbar = 0;

for i = 1:length(specificForce);
    
    
    
    %Leveling Process
    if i <countermax
        fbar = fbar + specificforce(i);       
    elseif i == countermax %Initialize
        
        fbar = fbar/countermax;
        
        %Number of states
        state.n = 17;

        %Initialize position
        state.r = [L;lambda;h];
        state.dr = [0;0;0];
        %Initialize velocity
        state.v = [0;0;0];
        %Initialize Orientation
        [theta,phi] = levelingProcess(fbar); % pitch,roll angles
        si = 0; % initial wander azimuth
        Twb = R_ZYX(si,theta,phi); %Initial orientation coordinate transformation %check this
        
        state.t = [si,theta,phi];
        state.T = Twb;
        state.sinSi = sin(si);
        state.cosSi = cos(si);
               
        %Initialize Bias
        state.b_a = [0.0;0.0;0.0];
        state.b_g = [0.0;0.0;0.0];
        
        %Initialize errors
        state.dx = zeros(state.n,1);

        %Initialize Parameters
        params.g = gravityModel_WGS84(L,h); %gravity model
        params.omegaE = 7.292115 * 10 ^ -5; % WGS 84 Earths angular rate [rad/s]
        params.cosL = cos(L);
        params.sinL = sin(L);
        params.Q = getQ_QuasiStationaryUnknown(state);
        params.R = getR_QuasiStationaryUnknown();
        
        

    else
        imu.f = specificForce(i);
        imu.omega = angularRate(i);
        imu.dt = dt(i);
        state = quasiStationaryAlignUnknownHeading(state,params,imu);            
    end
        
end

for i = 1:length(specificForce);
    
    if i == 1 %Initialize
                
        %Number of states
        state.n = 15;
        state.T = R_ZYX(state.t(1),state.t(2),state.t(3)); %Initial orientation coordinate transformation %check this
        state.r0 = state.r;
        %Initialize errors
        state.dx = zeros(state.n,1);
        
        %Initialize paramaters
        params.Q = getQ_QuasiStationaryZVU();
        params.R = getR_QuasiStationaryZVU();

    else
        imu.f = specificForce(i);
        imu.omega = angularRate(i);
        imu.dt = dt(i);
        state = quasiStationaryAlignZVU(state,params,imu);            
    end
        
end
        
 
    