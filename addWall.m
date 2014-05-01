function [mapret,optWallsret,Vret,Eret] = addWall(map,V,E,radius,optWalls,wall)
% ADDWALL: Adds a specified optional wall to the map, updating the set of
% nodes and edges accordingly
% 
%   ADDWALL(MAP,WAYPOINTS,STEP,N,RADIUS) returns 
%   updated map, optWalls, V and E variables
% 
%   INPUTS  
%       map         N-by-4 matrix containing the coordinates of walls in the 
%                   environment: [x1, y1, x2, y2]
%       V           Set of nodes currently in roadmap
%       E           Set of edges currently in roadmap
%       radius      radius of robot
%       optWalls    N-by-4 matrix containing the coordinates of optional 
%                   walls in the environment: [x1, y1, x2, y2]
%       wall        Integer index of wall in optWalls to add to map
%
%   OUTPUTS 
%       mapret      Updated set of walls in environment
%       optWallsret Updated set of optional walls
%       Vret        Updated set of nodes in roadmap
%       Eret        Updated set of edges in roadmap
%
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

%% ============================================================================
% INITIALIZE VARIABLES
%==============================================================================
% Get map dimensions
x_min = min(min(map(:,1)),min(map(:,3)));
x_max = max(max(map(:,1)),max(map(:,3)));
y_min = min(min(map(:,2)),min(map(:,4)));
y_max = max(max(map(:,2)),max(map(:,4)));
xRange = [x_min,x_max];
yRange = [y_min,y_max];

% Get coordinates of wall in optWalls to add to map
new_wall = optWalls(wall,:);

% Add new wall to map
mapret = [map;new_wall];
plot([new_wall(1),new_wall(3)],[new_wall(2),new_wall(4)],'k','LineWidth',3);

% Remove wall from optWalls
optWalls(wall,:) = [];
optWallsret = optWalls;

% Initialize Arrays to hold updated node and edge sets
Vret = [];
Eret = [];

% Initialize array to hold nodes that should be removed 
Vrem = [];

% Only add points that are still free to Vret
for i=1:size(V)
    if ptIsFree(V(i,:),mapret,radius,xRange,yRange)
        Vret = [Vret;V(i,:)];
    else
        Vrem = [Vrem;V(i,:)];
    end   
end

% Only add edges that are still free to Eret
for i=1:size(E)
    if(edgeIsFree(E(i,:),mapret,radius) && ~any(ismember([E(i,1:2);E(i,3:4)],Vrem,'rows')))
        Eret = [Eret;E(i,:)];
    end
end

end