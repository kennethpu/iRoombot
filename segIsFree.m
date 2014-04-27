function free = segIsFree(seg,map)
% SEGISFREE: Helper function to check if a line segment does not intersect 
% with any of the line segments defining walls in the environment
%
%   SEGISFREE(SEG,MAP) returns 
%   whether a line segment does not intersect with any walls in the map
% 
%   INPUTS 
%       seg   1-by-4 array containing 2 x/y coordinates of points defining
%             a line
%       map   N-by-4 matrix containing the coordinates of walls in the 
%             environment: [x1, y1, x2, y2]
% 
%   OUTPUTS 
%       free  boolean representing whether seg does not intersect with any
%             obstacles in the map

    % Initialize variables
    x1 = seg(1);
    y1 = seg(2);
    x2 = seg(3);
    y2 = seg(4);
    free = 1;
    
    % Check if seg intersects with any of the walls defined by map
    for i=1:size(map,1)
        x3 = map(i,1);
        y3 = map(i,2);
        x4 = map(i,3);
        y4 = map(i,4);
        isect = intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
        free = free & ~isect;
        if (~free)
            break;
        end
    end
end