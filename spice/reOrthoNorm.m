function R = reOrthoNorm(R)
%reOrthoNorm : Comptutes an orthonormal coordinate transformation matrix
%from a coordinate transformation matrix that may not be strictly
%orthonormal.
%
%Inputs
%   R : Coordinate Tranformation Matrix
%
%Outputs
%   R : Coordinate Tranformation Matrix
%
%Reference
%   Method described in Groves Chapter 5 : Inertial Navigation pg 141-142
%
%Log
% 3/5/17 Brandon Wood : Initial Implementation


c1m = R(:,1);
c2m = R(:,2);
c3m = R(:,3);

for i = 1:3
    for j = 1:3       
        D(i,j) = R(:,i)'*R(:,j);
    end
end

%Reorthogonalize
c1 = c1m - 0.5*(D(1,2)*c2m +D(1,3)*c3m);
c2 = c2m - 0.5*(D(1,2)*c1m +D(2,3)*c3m);
c3 = c3m - 0.5*(D(1,3)*c1m +D(2,3)*c2m);

%Renormalize
c1 = 2 / (1 + c1'*c1) * c1;
c2 = 2 / (1 + c2'*c2) * c2;
c3 = 2 / (1 + c3'*c3) * c3;

R = [c1,c2,c3];


end