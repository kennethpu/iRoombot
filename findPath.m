function [path,goal,gfound] = findPath(map,V,E,start,waypoints,ECwaypoints,radius)
% FINDPATH: Returns the shortest path between a start node and 
% the closest waypoint given a set of nodes V and edges E.
% 
%   [PATH,GOAL,GFOUND] = FINDPATH(MAP,V,E,START,WAYPOINTS,ECWAYPOINTS,RADIUS) returns
%   the shortest path between a start node and closest waypoint 
%   given a set of nodes V and edges E.
% 
%   INPUTS
%       map         N-by-4 matrix containing the coordinates of walls in the 
%                   environment: [x1, y1, x2, y2]
%       V           Set of nodes in roadmap
%       E           Set of edges in roadmap
%       start       1-by-2 array containing x/y coordinates of start location
%       waypoints   N-by-2 matrix containing the coordinates of waypoints in
%                   the environment: [x, y]
%       ECwaypoints N-by-2 matrix containing the coordinates of extra credit 
%                   waypoints in the environment: [x, y]
%       radius      radius of robot
% 
%   OUTPUTS
%       path        N-by-2 array containing a series of points representing the
%                   shortest path connecting initial and closest goal points
%       goal        1-by-2 array containing x/y coordinates of closest goal node
%       gfound      An integer denoting the number of goals found
%
%   Cornell University
%   MAE 4180/5180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295) 

%% ============================================================================
% INITIALIZE VARIABLES
%==============================================================================
% For all possible edges between start node and vertices in V, check if the
% edge is valid and if so add it to E
for i=1:size(V,1)
    v = V(i,:);
    if (edgeIsFree([start,v],map,radius))
        E = [E;start,v];
        plot([start(1),v(1)],[start(2),v(2)],'b');
    end
end

% Add start point to V
V = [V;start];

% Construct list of goal nodes 
goals = [waypoints;ECwaypoints];

% Find minimum path using dijkstras
[path,goal,gfound] = dijkstra(V,E,start,goals);

end