function [ h_pts ] = h_pack( pts )
%H_PACK Add homogeneous row to a array of points
    n = size(pts, 2);
    h_pts = [pts; ones(1, n)];

end
