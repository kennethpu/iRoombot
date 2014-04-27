function free = edgeIsFree(edge,map,radius)
% EDGEISFREE: Helper function to check if a edge does not intersect with 
% any of the line segments defining obstacles in the environment. Also 
% takes into account radius of robot by checking parallel segments to 
% either side of edge
%
%   EDGEISFREE(EDGE,MAP,RADIUS) returns 
%   whether or not an edge will not result in a collision
% 
%   INPUTS 
%       edge   1-by-4 array containing 2 x/y coordinates of points defining
%              a line
%       map    N-by-4 matrix containing the coordinates of walls in the 
%              environment: [x1, y1, x2, y2]
%       radius radius of robot
% 
%   OUTPUTS 
%       free   boolean representing whether edge (and parallel edges)
%              do not intersect with edges in map

    % Check edge directly
    free = segIsFree(edge,map);
    if (free)
        % Check parallel lines to either side of edge a radius distance away
        nsegs = parLines(edge,radius);
        seg1 = nsegs(1:4);
        free = free & segIsFree(seg1,map);
        seg2 = nsegs(5:8);
        free = free & segIsFree(seg2,map);
    end
end