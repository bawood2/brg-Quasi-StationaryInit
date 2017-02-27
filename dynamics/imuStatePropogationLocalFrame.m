function state = imuStatePropogationLocalFrame(state,params,imu)

%Initial values
dt = state.dt; %Timestep


hkm1 = state.r(3);
lambdakm1 = state.r(2);
Lkm1 = state.r(1);
vkm1 = state.v; % velocity
Tkm1 = state.T; % Coordiante Fram Tranformation

%Correct IMU measurements with bias estimates
omegaIB = imu.omega - state.b_g;
f = imu.f - state.b_a;

OmegaIE = get_OmegaIE(state,params);
OmegaEN = get_OmegaEN(state,params);
OmegaIB = skewmat(omegaIB); 
Tk = Tkm1*(eye(3) + OmegaIB*dt) - dt*(OmegaIE - OmegaEN)*Tkm1; % Chapter 5 pg 132  eq 5.39
state.T = reOrthoNorm(Tk);

%Propogate velocity
fw = 0.5* ( Tkm1 + state.T  ) * f;  % Chapter 5 pg 132  eq 5.40


state.v = vkm1 + ( fw +[0.0;0.0;params.g] - (OmegaEN + 2.0*OmegaIE)*vkm1 )*dt; % Chapter 5 pg 133 eq 5.47
%may need to use different value of g here

%Propogate position
sinLm = sin(Lkm1);
cosLm = cos(Lkm1);
REm = get_RE(sinLm,params);
RNm = get_RN(sinLm,params);
vNm = vkm1(1);
vEm = vkm1(2);
vDm = vkm1(3);
vNp = state.v(1);
vEp = state.v(2);
vDp = state.v(3);


hk = hkm1 - dt/2.0 * ( vDm + vDp);
Lk = Lkm1 + dt/2.0 * ( vNm / ( RNm+hkm1) + vNp / (Rnm + hk) );
sinLp = sin(Lk);
cosLp = cos(Lk);
REp = get_RE(sinLp,params);
lambdak = lambdakm1 + dt/2.0 * ( vEm / ( (REm + hkm1)*cosLm ) + vEp / ( (REp + hk)*cosLp ) );

state.r = [Lk;lambdak;hk];

end

function OmegaIE = get_OmegaIE(state,params)
%Chapter 2 pg 44 and Chapter 5 pg 131
L = state.r(1);
omegaIE = [params.omegaIE * cos(L) ; 0.0 ; params.omegaIE*sin(L)]; % eq 2.75
OmegaIE = skewmat(omegaIE); % eq 5.34

end

function OmegaEN = get_OmegaEN(state,params)
%Chapter 2 pg 41 and Chapter 5 pg 131

L = state.r(1);
h = state.r(3);
vN = state.v(1);
vE = state.v(2);
sinL = sin(L);
cosL = cos(L);
tanL = sinL/cosL;
RN = get_RN(sinL,params);
RE = get_RE(sinL,params);
rnh = ( RN + h);
reh = ( RE + h);

omegaEN = [ vE/reh ; -vN/rnh ; -vE*tanL/reh];
OmegaEN = skewmat(omegaEN); %Chapter 5 pg 131 eq 5.37


end

function RN = get_RN(sinL,params)
%Chapter 2 pg 41
RN = params.R0 * ( 1.0 - params.e^2) / (( 1.0 - params.e^2 * sinL^2)^(3.0/2.0));

end

function RE = get_RE(sinL,params)
%Chapter 2 pg 41
RE = params.R0 / (( 1.0 - params.e^2 * sinL^2)^(1.0/2.0));

end

