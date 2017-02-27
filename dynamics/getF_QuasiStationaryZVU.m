function F = getF_QuasiStationaryZVU(state,params,imu)

O = zeros(3,3);

T = state.T; % Coordinate transformation matrix
f = imu.f - state.b_a; % specific force
g = params.g; %May need to update this with current state estimate
wie = params.omegaIE;

L = state.r(1);
h = state.r(3);
vN = state.v(1);
vE = state.v(2);
vD = state.v(3);

sinL = sin(L);
cosL = cos(L);
tanL = sinL/cosL;
sec2L = (1.0/cosL)^2;
RN = get_RN(sinL,params);
RE = get_RE(sinL,params);
reS = get_reS(sinL,cosL,RE,params);

rnh = ( RN + h);
reh = ( RE + h);

%% Compute position Jocobian
f11 = 1.0 / rnh;
f22 = 1.0 / (reh*cosL);
f33 = -1.0;
F12 = [ f11,0.0,0.0;...
        0.0,f22,0.0;...
        0.0,0.0,f33];
    
f13 = - vN / rnh^2;
f21 =  vE*sinL / (reh*cosL^2);
f23 = vE / (reh^2 * cosL);
F13 = [ 0.0,0.0,f13;...
        f21,0.0,f23;...
        0.0,0.0,0.0];
    
drdot_dx = [ O F12 F13 O O];


%% Compute velocity Jocobian

F21 = - skewmat(T*f);

f11 = vD / rnh;
f12 = -2.0*vE*tanL/reh - 2.0*wie*sinL;
f13 = vN / rnh;

f21 = vE*tanL/reh + 2.0*wie*sinL;
f22 = (vN*tanL +vD)/reh;
f23 = vE/reh +2.0*wie*cosL;

f31 = -2.0*vN/rnh;
f32 = -2.0*vE/reh -2.0*wie*cosL;


F22 = [ f11,f12,f13;...
        f21,f22,f23;...
        f31,f32,0.0];

f11 = -vE^2*sec2L/reh -2.0*vE*wie*cosL;
f13 = -vE^2*tanL/reh^2 - vN*vD/rnh^2;
f21 = vN*vE*sec2L/reh + 2.0*vN*wie*cosL - 2.0*vD*wie*sinL;
f23 = (-vN*vE*tanL+vE*vD)/reh^2;
f31 = 2.0*vE*wie*sinL;
f33 = (vE/reh)^2 + (vN/rnh)^2 - 2.0*g/reS;


F23 = [ f11,0.0,f13;...
        f21,0.0,f23;...
        f31,0.0,f33];


dvdot_dx = [F21 F22 F23 T O];

%% Compute attitude Jacobian
omegaIE = [wie*cosL; 0.0 ; -wie*sinL]; %Chapter 2 pg 44
omegaEN = [ vE/reh ; -vN/rnh ; -vE*tanL/reh]; %Chapter 5 pg 131
omegaIN = omegaIE + omegaEN;
F31 = -skewmat(omegaIN);

f12 = -1.0 /reh;
f21 = 1.0/rnh;
f32 = tanL/reh;
F32 = [ 0.0,f12,0.0;...
        f21,0.0,0.0;...
        0.0,f32,0.0];
    
f11 = wie*sinL;
f13 = vE / reh^2;
f23 = -vN/rnh^2;
f31 = wie*cosL + vE/(reh*cosL^2);
f33 = -vE*tanL/reh^2;
F33 = [ f11,0.0,f13;...
        0.0,0.0,f23;...
        f31,0.0,f33];

dsidot_dx = [F31 F32 F33 O T];

%Compute accelerometer bias Jacobain
dba_dx = [O O O O O];

%Compute gyro bias Jacobian
dbg_dx = [O O O O O];

%Compute F
F = [ drdot_dx ; dvdot_dx ; dsidot_dx ; dba_dx ; dbg_dx ];

end

function reS = get_reS(sinL,cosL,RE,params)
%Chapter 2 pg 48
reS = RE*sqrt(cosL^2 +(1 - params.e^2)^2 *sinL^2);
end

function RN = get_RN(sinL,params)
%Chapter 2 pg 41
RN = params.R0 * ( 1.0 - params.e^2) / (( 1.0 - params.e^2 * sinL^2)^(3.0/2.0));

end

function RE = get_RE(sinL,params)
%Chapter 2 pg 41
RE = params.R0 / (( 1.0 - params.e^2 * sinL^2)^(1.0/2.0));

end