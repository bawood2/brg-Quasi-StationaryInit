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

%Lat Long coordinates are for Coordinated Science Lab (Google Earth)
L = -88.22675*DEG_TO_RAD; % longitude   88°13'36.30"W 
lambda = 40.114742*DEG_TO_RAD; %latitude   40° 6'53.07"N
h = 222; %altitude  %	222 m elevation of UIUC 

%In system, IMU measurements are being taken.  Prior to alignment, need to
%initialize orientation.  This is done by a leveling process. Prior to
%this, a time average of IMU is taken.

counter = 1;
countermax = 500;
fbar = [0;0;0];
wbar = [0;0;0];

%create burn in/out index due to exclude errors at begin and end of data
burnIn = 1000;  %corresponds to 5 seconds
burnOut = 1000; %corresponds to 5 seconds 
countermax = countermax+ burnIn;

%Initialize Data Structures
state = [];
imu = [];
params = [];

for i = burnIn:(lenTime-burnOut);
    
    
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
        
        %si = 0;
        state.t = [si;theta;phi];
        
        state.sinSi = sin(si);
        state.cosSi = cos(si);
        
        %state.sinSi = 0;
        %state.cosSi = 0;
        
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
        params = gravityModel_WGS84(L,h,params); %gravity model
        params.cosL = cos(L);
        params.sinL = sin(L);
        params.Q = getQ_MotorolaMotoZPlayUnknown(state);
        params.R = getR_QuasiStationaryUnknown();
        params.G = getG_QuasiStationaryUnknown(state);
        
        %For debugging
        params.loopCount = 0;
        
        

    else
        tk = systemTime(i);
        ak = find_closest_index(accelData(:,1),tk);
        gk = find_closest_index(gyroData(:,1),tk);    
        imu.f = accelData(ak,2:4)';
        imu.omega = gyroData(gk,2:4)';
        imu.dt = 0.005 ; %dt(i);
        state = quasiStationaryAlignUnknownHeading(state,params,imu);  
        
        params.loopCount = params.loopCount+1;
    end
    
    
    
end

% Old Implementation : Commented out 3/16/2017
% %Once wander azimuth solution <2deg...
%     %Make consistant with (cosSi)^2 + (sinSi)^2 = 1
%     cosSi  =  state.cosSi;
%     sinSi = 1 - cosSi^2;
%     cosSi = 1 - sinSi^2;
%     %Compute transformation from local navigation to body
%     Rnw = [ cosSi, -sinSi, 0.0; sinSi, cosSi, 0.0; 0.0, 0.0, 1.0];
%     Tnb = Rnw*state.T;
%     Tnb = reOrthoNorm(Tnb);
%     state.T = Tnb;
%     %Tranform resolving frame from wander to local navigation
%     state.r = Rnw*state.r;
%     state.v = Rnw*state.v;
%     state.t = getEulerZYX(state.T);  %get new attitude

%Start QuasiStationary Zero Velocity Update

for i = burnIn:(lenTime-burnOut);
    
    if i == burnIn %Initialize
                
        %Number of states
        state.n = 15;
        
        % Initialize quasi-stationary zero velocity update
        state.r = [L;lambda;h]; % reinitialize position
        state.r0 = [L;lambda;h];
        state.v = [0;0;0];  % reinitialize velocity
        state.t = rot2euler_zyx(state.T);  % Retrieve Euler Angle Sequence 

        %Initialize errors
        state.dx = zeros(state.n,1);
        
        %For now, keep covariance
        state.P(10:11,:) = [];
        state.P(:,10:11) = [];
        %state.P = getP0_QuasiStationaryZVU();

        %Initialize Parameters
        params = gravityModel_WGS84(L,h); %gravity model
        params.Q = getQ_MotorolaMotoZPlay();
        params.R = getR_QuasiStationaryZVU();
        params.G = getG_QuasiStationaryZVU();
        
        %For debugging
        params.loopCount = 0;
    end
    
        tk = systemTime(i);
        ak = find_closest_index(accelData(:,1),tk);
        gk = find_closest_index(gyroData(:,1),tk);    
        imu.f = accelData(ak,2:4)';
        imu.omega = gyroData(gk,2:4)';
        imu.dt = 0.005 ; %dt(i);
        state = quasiStationaryAlignZVU(state,params,imu);       
        
        %Update Gravity with new position estimate
        params = gravityModel_WGS84(state.r(1),state.r(3),params); %gravity model
        
        params.loopCount = params.loopCount+1; 
        
end
