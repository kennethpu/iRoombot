function free = ptIsFree(pt,map,radius,xRange,yRange)
% PTISFREE: Helper function to check if a point is not within radius 
% distance away from any of the edges defined by map
% 
%   PTISFREE(PT,MAP,RADIUS,XRANGE,YRANGE) returns 
%   whether or not a point will not result in a collision
%
%   INPUTS 
%       pt     1-by-2 array containing x/y coordinates of test point
%       map    N-by-4 matrix containing the coordinates of walls in the 
%              environment: [x1, y1, x2, y2]
%       radius radius of robot
%       xRange 1-by-2 vector representing interval of possible X values
%       yRange 1-by-2 vector representing interval of possible Y values
% 
%   OUTPUTS 
%       free   boolean representing whether point is more than radius 
%              away from all edges
%
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

    % Initialize variables
    px = pt(1);
    py = pt(2);
    
    % Check that point is contained within x and y range
    free = (xRange(1)<=px)&&(px<=xRange(2))&&(yRange(1)<=py)&&(py<=yRange(2)); 
    
    % Check if point is more than a minimum distance away from walls in map
    if (free)
        for i=1:size(map,1)
            p1 = map(i,1:2);
            p2 = map(i,3:4);
            d = distToSegment(pt,[p1,p2]);
            free = free & (d>radius);
            if (~free)
                break;
            end
        end
    end
end