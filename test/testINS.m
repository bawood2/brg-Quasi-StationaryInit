clear
close all
set(0,'DefaultFigureWindowStyle','docked')
%Plots
plot_text_size = 14;
plot_axis_width = 1.1;
markersize = 20;
linewidth = 2;
saveflag = 1;

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

%create burn in/out index due to exclude errors at begin and end of data
burnIn = 1000;  %corresponds to 5 seconds
burnOut = 1000; %corresponds to 5 seconds 
countermax = countermax+ burnIn;

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
        
        
        state.t = [si;theta;phi];
        
        state.sinSi = sin(si);
        state.cosSi = cos(si);
        %Twn = [state.cosSi, state.sinSi, 0.0; -state.sinSi, state.cosSi, 0.0; 0.0,0.0,1.0];
        Twb = R_ZYX(si,theta,phi); %Initial orientation coordinate transformation %check this
        %state.T = Twn*Tnb;
        state.T = Twb; 
        
        
        %Initialize Bias
        state.b_a = 1.0e-03 *[0.0738;-0.0427;-0.1063];
        state.b_g = 1.0e-03 *[-0.3541;-0.0131;0.5161];
        
        %Initialize errors
        state.dx = zeros(state.n,1);
        state.P = getP0_QuasiStationaryUnknown();

        %Initialize Parameters
        params = gravityModel_WGS84(L,h); %gravity model
        params.cosL = cos(L);
        params.sinL = sin(L);
        params.Q = getQ_MotorolaMotoZPlayUnknown(state);
        params.R = getR_QuasiStationaryUnknown();
        params.G = getG_QuasiStationaryUnknown(state);
        
        %For debugging
        params.loopCount = 1;
        store.t0 = systemTime(i);
  
    else
        tk = systemTime(i);
        ak = find_closest_index(accelData(:,1),tk);
        gk = find_closest_index(gyroData(:,1),tk);    
        imu.f = accelData(ak,2:4)';
        imu.omega = gyroData(gk,2:4)';
        imu.dt = 0.005 ; %dt(i);
        state = imuStatePropogationUnknown(state,params,imu);
        %state = quasiStationaryAlignUnknownHeading(state,params,imu);  
        
        %Insert function to get Euler Angles from Coordinate transformation
        %matrix
        
        store.state(params.loopCount) = state;
        store.t(params.loopCount) = tk;
        store.theta(:,params.loopCount) = rot2euler_zyx(state.T);
               
        params.loopCount = params.loopCount+1;
    end
    
    
    
end

dr = [store.state.dr];
dv = [store.state.v];
theta = [store.theta];
t = (store.t-store.t0)/1000;

%Plot Position
for i = 1:3
    
figure(1)
plot(t,dr(i,:),'Linewidth',linewidth);
hold on

figure(2)
plot(t,dv(i,:),'Linewidth',linewidth);
hold on

figure(3)
plot(t,theta(i,:),'Linewidth',linewidth);
hold on

end

figure(1)
xlabel('Time');
ylabel('X');
legend('x-position','y-position','z-position')

figure(2)
xlabel('Time');
ylabel('V');
legend('v_x','v_y','v_z')

figure(3)
xlabel('Time');
ylabel('\theta');
legend('yaw','roll','pitch')

