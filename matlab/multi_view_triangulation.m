function [ pobj1, pobj2, erreur ] = multi_view_triangulation( T12, pt1, pt2 )
%triangulation Obtain a physical point coordinate given a pair of
%points in metric coordinates and their extrinsic matrix
%   T12: Extrinsic transformation between cameras 1 and 2
%   pt1: 3x1 point in metric coordinates expressed in camera 1 frame
%   pt2: 3x1 point in metric coordinates expressed in camera 2 frame
%   pobj1: 3x1 Output physical point coordinate in camera 1 frame
%   pobj2: 3x1 Output physical point coordinate in camera 2 frame
R12 = T12(1:3, 1:3);
t12 = T12(1:3, 4);
% express the direction vector reliying the center of the cameras and the
% points in camera 1 frame
origin1 = zeros(3, 1);
dir1 = pt1;
% dir1xz = [dir1(1) dir1(3)]; % on enleve l'y
origin2 = t12;
dir2 = R12*pt2;
% dir2xz = [dir2(1) dir2(3)]; % on enleve l'y

% Solve the linear system for triangulation Ax = B
A = [dir1 -dir2];
B = origin2-origin1;
% B = [B(1) B(3)];
x = A\B; % inv(A)*B
pobj1 = origin1 + x(1)*dir1;
pobj2 = h_unpack(inv(T12)*h_pack(pobj1));
erreur = norm(origin1 + x(1)*dir1 - (origin2 + x(2)*dir2));
end