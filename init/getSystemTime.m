function systemTime = getSystemTime(accelData,gyroData)

%Get initial and final times
t0_a = accelData(1,1);
tf_a = accelData(end,1);
t0_g = gyroData(1,1);
tf_g = gyroData(end,1);

%Get average data rate
aTime = accelData(:,1)/1000.0; %Unix time [s] of accel measurements
dta = aTime(2:end) - aTime(1:(end-1));
gTime = gyroData(:,1)/1000.0;
dtg = gTime(2:end) - gTime(1:(end-1));
dt = (mean(dta) + mean(dtg))/2.0;  %take average time for now.  

t0 = max(t0_a,t0_g);
tf = min(tf_a,tf_g);

%Create System Time Array
systemTime = (t0:dt*1000:tf);

end