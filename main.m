%quasiStationaryAlignUnknownHeading test script
DEG_TO_RAD = pi/180.0;


%Load data
accelFID = 'quasi-stationary_accel.xlsx';
gyroFID = 'quasi-stationary_gryo.xlsx';
magFID = 'quasi-stationary_mag.xlsx';
accelData = xlsread(accelFID);
gyroData = xlsread(gyroFID);
magData = xlsread(magFID);

% Initialize Data.  These should all be inputs
specificForce = accelData(:,2:4); % Load IMU specific force measurements
angularRate = gyroData(:,2:4); % Load IMU angular rate measurements

%Get the average rate of each measurement
% 3/3/17 : May need to change this to searching for specific times since
% the recorded times between the measurements do not align.  

aTime = accelData(:,1)/1000.0; %Unix time [s] of accel measurements
gTime = gyroData(:,1)/1000.0;
dt = (mean(aTime) + mean(gTime))/2.0;  %take average time for now.  

%Get the number of measurements
na = length(aTime);
ng = length(gTime);
nMeasurements = (na<ng)*na + (ng<na)*ng; 

%Lat Long coordinates are for UIUC
L = 88.2272*DEG_TO_RAD; % longitude  88.2272° W  
lambda = 40.1020*DEG_TO_RAD; %latitude  40.1020° N
h = 225; %altitude  %	225 m elevation of champaign IL



%In system, IMU measurements are being taken.  Prior to alignment, need to
%initialize orientation.  This is done by a leveling process. Prior to
%this, a time average of IMU is taken.

counter = 1;
countermax = 20;
fbar = 0;

for i = 1:length(nMeasurements);
    
    
    
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
    
    %Once wander azimuth solution <2deg...
    %Make consistant with (cosSi)^2 + (sinSi)^2 = 1
    cosSi  =  state.cosSi;
    sinSi = 1 - cosSi^2;
    cosSi = 1 - sinSi^2;
    %Compute transformation from local navigation to body
    Rnw = [ cosSi, -sinSi, 0.0; sinSi, cosSi, 0.0; 0.0, 0.0, 1.0];
    Tnb = Rnw*state.T;
    Tnb = reOrthoNorm(Tnb);
    state.T = Tnb;
    %Tranform resolving frame from wander to local navigation
    state.r = Rnw*state.r;
    state.v = Rnw*state.v;
    state.t = getEulerZYX(state.T);  %get new attitude

    
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
        
 
    