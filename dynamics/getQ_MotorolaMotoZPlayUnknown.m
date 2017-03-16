function Q = getQ_MotorolaMotoZPlayUnknown(state)
%getQ_QuasiStationaryUnknown : get continuous process noise covaraince 
% matrix for the for the Motorola Moto Z Play smartphone
%Inputs
%
%Outputs
%   Q : Continuous Process Noise Covaraince
%
%Reference
%   Equation from Groves Chapter 12 pg 387
%   MagPIE: A Dataset for Positioning with Magnetic Anomolies (D. Hanley et
%   al.)  Used to estimate Allan Variance
%   Notes on Stochastic Errors of Low Cost MEMS Inertial Units: (Y. Yuksel &
%   H.B.Kaygisiz)  Used to compute variances
%Log 
% 3/16/17 Brandon Wood : Initial Implementation

O = zeros(3,3);

%Approximate values from 
%Characterizing Stochastic Errors of MEMS – BasedInertial Sensors
%Pham Van Tang1,*, Tran Duc Tan2, Chu Duc Trinh2 
%VNU Journal of Science: Mathematics – Physics, Vol. 32, No. 2 (2016) 34-42

sampleRate = 200; %Assume no downsampling.  Sample rate of Motorola Phone 200Hz

%Accelerometer Variance
%Numerator variance values taken from Allan Variance curve at tau = 1
qax = (8.5*10^-5 / sampleRate)^2;
qay = (7.5*10^-5 / sampleRate)^2;
qaz = (4.75*10^-5 / sampleRate)^2;
qa = diag([qax;qay;qaz]);

%Gyro Variance
%Numerator variance values taken from Allan Variance curve at tau = 1
qgx = (0.4*10^-8 / sqrt(sampleRate))^2;
qgy = (1.4*10^-8 / sqrt(sampleRate))^2;
qgz = (2.0*10^-8 / sqrt(sampleRate))^2;
qg = diag([qgx,qgy,qgz]);

%tauPeak : time of flicker noise
%sigmaPeak : Allen Variance at time of flicker noise

%Accelerometer Bias Variance
tauPeak = 150;
sigmaPeak = 8.25*10^-6;
qbax = firstOrderMarkov(tauPeak,sigmaPeak,sampleRate);
tauPeak = 80;
sigmaPeak = 9.25*10^-5;
qbay = firstOrderMarkov(tauPeak,sigmaPeak,sampleRate);
tauPeak = 110;
sigmaPeak = 2.10*10^-6;
qbaz = firstOrderMarkov(tauPeak,sigmaPeak,sampleRate);
qba = diag([qbax qbay qbaz]);

%Gyro Bias Variance
tauPeak = 180;
sigmaPeak = 9.75*10^-8;
qbgx = firstOrderMarkov(tauPeak,sigmaPeak,sampleRate);
tauPeak = 150;
sigmaPeak = 8.4*10^-8;
qbgy = firstOrderMarkov(tauPeak,sigmaPeak,sampleRate);
tauPeak = 125;
sigmaPeak = 0.75*10^-9;
qbgz = firstOrderMarkov(tauPeak,sigmaPeak,sampleRate);
qbg = diag([qbgx qbgy qbgz]);

Q = [ O O O o o O O;...
      O qa O o o O O;...
      O O qg o o O O;...
      zeros(2,state.n);...
      O O O o o qba O;...
      O O O o o O qbg];

end

function q = firstOrderMarkov(tauPeak,sigmaPeak,sampleRate)
dt = 1/sampleRate;
Tc = tauPeak/1.89;
qc = sigmaPeak / 0.437 / sqrt(Tc);

q = qc / (2.0)*Tc * (1 - exp(dt/Tc));

end