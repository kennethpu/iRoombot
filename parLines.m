function nsegs = parLines(seg,dist)
% PARLINES: Helper function to find two lines that are parallel to a 
% given line segment and a given distance away in either direction 
% from the given line
%
%   PARLINES(SEG,DIST) returns 
%   two line segments parallel to given line segment that are a 
%   specified distance away in either direction
% 
%   INPUTS 
%       seg   1-by-4 array containing 2 x/y coordinates of points defining
%             a line segment
%       dist  perpindicular distance between each new line and given line
% 
%   OUTPUTS 
%       nsegs 1-by-8 array containing 4 x/y coordinates of points defining
%             2 line segments

    % Get segment endpoints
    e1 = seg(1:2);
    e2 = seg(3:4);
    
    % Calculate normalized directional vector
    v = e2 - e1;
    v = v/norm(v);
    
    % Get perpindicular unit vector
    v_p = [-v(2) v(1)];
    
    % Calculate endpoints of two parallel line segments given distance to
    % either side
    a1 = e1 + dist.*v_p;
    a2 = e2 + dist.*v_p;
    b1 = e1 - dist.*v_p;
    b2 = e2 - dist.*v_p;
    nsegs = [a1 a2 b1 b2];
end


