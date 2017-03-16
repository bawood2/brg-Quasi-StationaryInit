function state = imuStatePropogationUnknown(state,params,imu)

%Initial values
dt = imu.dt; %Timestep

vkm1 = state.v; % velocity
Tkm1 = state.T; % Coordiante Frame Transformation

%Correct IMU measurements with bias estimates
omegaIB = imu.omega - state.b_g;
f = imu.f - state.b_a;

M = [ 0 , params.sinL , -state.sinSi* params.cosL;...
    -params.sinL, 0, - state.cosSi* params.cosL;...
    state.sinSi* params.cosL, state.cosSi* params.cosL, 0];

OmegaIB = skewmat(omegaIB); 
Tk = Tkm1*(eye(3) + OmegaIB*dt) - params.omegaIE*dt*M*Tkm1; % eq13.12  
%2/26/17 : may be an error with this equation.  Omega may need to be Omega*dt
state.T = reOrthoNorm(Tk);

%Propogate velocity
fw = 0.5* ( Tkm1 + state.T  ) * f;
state.v = vkm1 + (fw +[0;0;params.g])*dt; % eq13.13

%Propogate delta position
state.dr = state.dr + 0.5*dt*(vkm1+state.v);





end
