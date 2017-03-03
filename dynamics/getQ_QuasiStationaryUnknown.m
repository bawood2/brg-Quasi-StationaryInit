function Q = getQ_QuasiStationaryUnknown(state)

O = zeros(3,3);
o = zeros(3,1);

%Approximate values from 
%Characterizing Stochastic Errors of MEMS – BasedInertial Sensors
%Pham Van Tang1,*, Tran Duc Tan2, Chu Duc Trinh2 
%VNU Journal of Science: Mathematics – Physics, Vol. 32, No. 2 (2016) 34-42
qax = (0.005)^2;
qay = (0.008)^2;
qaz = (0.01)^2;
qa = diag([qax;qay;qaz]);

qgx = (0.001)^2;
qgy = (0.001)^2;
qgz = (0.0007)^2;
qg = diag([qgx,qgy,qgz]);

qbax = (0.001)^2;
qbay = (0.001)^2;
qbaz = (0.001)^2;
qba = diag([qbax qbay qbaz]);

qbgx = (0.0004)^2;
qbgy = (0.0004)^2;
qbgz = (0.0004)^2;
qbg = diag([qbgx qbgy qbgz]);

Q = [ O O O o o O O;...
      O qa O o o O O;...
      O O qg o o O O;...
      zeros(2,state.n);...
      O O O o o qba O;...
      O O O o o O qbg];

end