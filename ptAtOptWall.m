function [wall,isOpt] = ptAtOptWall(pt,map,optWalls)
%% ============================================================================
% ptAtOptWall
%==============================================================================
%   Helper function to get the closest wall to a given point and determine
%   if it is an optional wall or not
%
%   INPUTS 
%       pt          1-by-2 array containing x/y coordinates of test point
%       map         N-by-4 matrix containing the coordinates of walls in the 
%                   environment: [x1, y1, x2, y2]
%       optWalls    N-by-4 matrix containing the coordinates of optional 
%                   walls in the environment: [x1, y1, x2, y2]
% 
%   OUTPUTS 
%       wall        Integer index of wall in optWalls if minimum wall is
%                   optional
%       isOpt       Boolean flag denoting whether wall is optional
%
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

%% ============================================================================
% INITIALIZE VARIABLES
%==============================================================================
    min_wall = [];
    min_dist = 9999;
    
%% ============================================================================
% FIND CLOSEST WALL TO POINT
%============================================================================== 
    % Search through optional walls
    for i=1:size(optWalls,1)
        dist = distToSegment(pt,optWalls(i,:));
        if (dist < min_dist)
            min_dist = dist;
            min_wall = optWalls(i,:);
        end
    end
    
    % Search through walls in map
    for i=1:size(map,1)
        dist = distToSegment(pt,map(i,:));
        if (dist < min_dist)
            min_dist = dist;
            min_wall = map(i,:);
        end
    end
    
    % Check if closest wall is in optWalls
    [isOpt,wall] = ismember(min_wall,optWalls,'rows');

end