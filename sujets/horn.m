function [ T, R, t, reproj_error ] = horn( Pcam, Pobj )
%HORN 3D localization algorithm (pag 106 - 108)
% Pcam: 3xN Points in camera coordinate frame (mm/mm/mm)
% Pobj: 3XN Points in object coordinate frame (mm/mm/mm)
%
% Obs: input points are all in mm!
% if you need to project them using the intrinsics matrix, do it before
% calling the funtion.
% eg.
%   P_mm = h_unpack(inv(K)*h_pack(P_px));
%
% Procedure:
%   1. Compute both frames points barycenter
%   2. Compute optimal rotation 
%   3. Compute optimal translation
%   4. Extra: reprojection error

%% Handy (anonymous) functions
broadcast = @(vec, N) repmat(vec, 1, N); % broadcast into cols

%% Barycentric coordinates in sensor and object frames
% gravity centers
Cg = mean(Pcam, 2); % camera frame point barycenter
Og = mean(Pobj, 2); % object frame point barycenter
Fc = Pcam - broadcast(Cg, size(Pcam, 2)); % sum(Fc, 2) = 0
Fo = Pobj - broadcast(Og, size(Pobj, 2)); % sum(Fo, 2) = 0

%% Optimal rotation (assume det > 0)
M = Fc*Fo';
[U, ~, V] = svd(M);
R = U*V';
%% in case of reflection
if det(R) < 0
    R = U*[1 0 0; 0 1 0; 0 0 -1]*V';
    disp('Warning: transformation is a reflection');
end
%% Optimal translation
t = Cg - R*Og;
%% 3D transformation
T = [R t;
    zeros(1,3) 1];
%% Verification (RMS error)
err = Pcam - h_unpack(T*h_pack(Pobj));
reproj_error = sqrt(mean(err.*err, 2));

end

