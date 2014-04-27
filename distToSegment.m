function dist = distToSegment(pt,seg)
% DISTTOSEGMENT: Helper function to determine the minimum distance of a
% point from a line segment
% 
%   DISTTOSEGMENT(PT,SEG) returns 
%   the minimum distance of a point and a line segment
%
%   INPUTS 
%       pt    1-by-2 array containing x/y coordinates of test point
%       seg   1-by-4 array containing 2 x/y coordinates of points defining
%             a line segment
% 
%   OUTPUTS 
%       dist  minimum distance between a point and a line segment
%
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

    % Get endpoints of line segment
    e1 = seg(1:2);
    e2 = seg(3:4);
    
    % Get length squared of line segment
    l_sq = norm(e2-e1)^2;
    
    % If length squared is equal to 0, e1 = e2, simply find distance
    % between point and e1
    if (l_sq == 0)
        dist = pdist([pt;e1],'euclidean');
    else
        % Find projection of point onto line defined by e1,e2
        t = dot(pt-e1,e2-e1)/l_sq;
        if (t < 0)
            % Projection of point falls before e1, find distance between pt
            % and e1
            dist = pdist([pt;e1],'euclidean');
        elseif(t > 1)
            % Projection of point falls after e2, find distance between pt
            % and e2
            dist = pdist([pt;e2],'euclidean');
        else
            % Projection of point falls on line segment, find distance between pt
            % and projected point
            proj_pt = e1 + t*(e2-e1);
            dist = pdist([pt;proj_pt],'euclidean');
        end
    end
end