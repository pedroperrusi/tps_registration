function [ res ] = h_pack( v )
%H_PACK Summary of this function goes here
%   Detailed explanation goes here
res = [v; ones(1, size(v,2))];

end

