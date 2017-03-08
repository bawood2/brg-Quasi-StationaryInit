%quasiStationaryAlignUnknownHeading test script
DEG_TO_RAD = pi/180.0;


%Load data
accelFID = 'quasi-stationary_accel.xlsx';
gyroFID = 'quasi-stationary_gryo.xlsx';
magFID = 'quasi-stationary_mag.xlsx';
accelData = xlsread(accelFID);
gyroData = xlsread(gyroFID);
magData = xlsread(magFID);

%Create System Time
systemTime = getSystemTime(accelData,gyroData);
lenTime = length(systemTime);

%Lat Long coordinates are for UIUC
L = -88.2272*DEG_TO_RAD; % longitude  88.2272° W  
lambda = 40.1020*DEG_TO_RAD; %latitude  40.1020° N
h = 222; %altitude  %	222 m elevation of UIUC 

%lat / long champaign
%40.116421  
%-88.243385

%In system, IMU measurements are being taken.  Prior to alignment, need to
%initialize orientation.  This is done by a leveling process. Prior to
%this, a time average of IMU is taken.

counter = 1;
countermax = 500;
fbar = [0;0;0];
wbar = [0;0;0];

for i = 1:lenTime;
    
    
    %Leveling Process
    if i <countermax
        tk = systemTime(i);
        ak = find_closest_index(accelData(:,1),tk);
        gk = find_closest_index(gyroData(:,1),tk);
        f = accelData(ak,2:4)';
        w = gyroData(gk,2:4)';
        fbar = fbar + f;
        wbar = wbar + w;
    elseif i == countermax %Initialize
        
        fbar = fbar/countermax;
        wbar = wbar/countermax;
        
        %Number of states
        state.n = 17;

        %Initialize position
        state.r = [L;lambda;h];
        state.dr = [0;0;0];
        %Initialize velocity
        state.v = [0;0;0];
        %Initialize Orientation
        [theta,phi] = levelingProcess(fbar); % pitch,roll angles
        si = gyrocompassingProcess(wbar,theta,phi); % initial wander azimuth
        %Tnb = R_ZYX(0,theta,phi); %Initial orientation coordinate transformation %check this
        
        
        state.t = [si;theta;phi];
        
        state.sinSi = sin(si);
        state.cosSi = cos(si);
        %Twn = [state.cosSi, state.sinSi, 0.0; -state.sinSi, state.cosSi, 0.0; 0.0,0.0,1.0];
        Twb = R_ZYX(si,theta,phi); %Initial orientation coordinate transformation %check this
        %state.T = Twn*Tnb;
        state.T = Twb; 
        
        
        %Initialize Bias
        state.b_a = [0.0;0.0;0.0];
        state.b_g = [0.0;0.0;0.0];
        
        %Initialize errors
        state.dx = zeros(state.n,1);
        state.P = getP0_QuasiStationaryUnknown();

        %Initialize Parameters
        params = gravityModel_WGS84(L,h); %gravity model
        params.cosL = cos(L);
        params.sinL = sin(L);
        params.Q = getQ_QuasiStationaryUnknown(state);
        params.R = getR_QuasiStationaryUnknown();
        
        %For debugging
        params.loopCount = 0;
        
        

    else
        tk = systemTime(i);
        ak = find_closest_index(accelData(:,1),tk);
        gk = find_closest_index(gyroData(:,1),tk);    
        imu.f = accelData(ak,2:4)';
        imu.omega = gyroData(gk,2:4)';
        imu.dt = dt ; %dt(i);
        state = quasiStationaryAlignUnknownHeading(state,params,imu);  
        
        params.loopCount = params.loopCount+1;
    end
    
    
    
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
        
 
    