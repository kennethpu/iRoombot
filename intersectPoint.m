function[isect,x,y,ua]= intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4)
% INTERSECTPOINT: find the x/y coordinates of the intersection of two line
% segments (if intersection exists)
% 
%   [ISECT,X,Y,UA] = INTERSECTPOINT(X1,Y1,X2,Y2,X3,Y3,X4,Y4) calculates the
%   intersection point of two line segments
% 
%   INPUTS
%       x1,y1   x/y coordinates of point 1 on line segment 1
%       x2,y2   x/y coordinates of point 2 on line segment 1
%       x3,y3   x/y coordinates of point 1 on line segment 2
%       x4,y4   x/y coordinates of point 2 on line segment 2
% 
%   OUTPUTS
%       isect   bool variable (true if segments intersect, false otherwise)
%       x,y     x/y coordinates of intersection point (if isect = true)
%       ua      distance of intersection point along line 1 (0 <= ua <= 1) 
%               where: x = x1 + ua (x2 - x1) 
% 
%   Refer to http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d for
%   line segment intersection equations.
% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots

x = [];
y = [];
ua = [];

denom = (y4-y3)*(x2-x1)-(x4-x3)*(y2-y1);

if denom == 0
    % if denom = 0, lines are parallel
    isect = false;
    return;
else
    ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3))/denom;
    ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3))/denom;
    
    % if (0 <= ua <= 1) and (0 <= ub <= 1), intersection point lies on line
    % segments
    if ua >= 0 && ub >= 0 && ua <= 1 && ub <= 1
        isect = true;
        x = x1 + ua*(x2-x1);
        y = y1 + ua*(y2-y1);
    else
        % else, the intersection point lies where the infinite lines
        % intersect, but is not on the line segments
        isect = false;
        return;
    end
end

