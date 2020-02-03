function [ pts ] = h_unpack( h_pts )
%H_UNPACK Remove homogeneous row without normalization
%   h_pts: 3xN or 4xN vector of points with last homogeneous row
dim = size(h_pts, 1); % points dimension (2D/3D)
pts = h_pts(1:dim-1, :);

end