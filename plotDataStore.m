function plotDataStore(dataStore,optWalls,beaconLoc,waypoints,ECwaypoints)
% PLOTDATASTORE: Plots the provided dataStore variables, including the
% final map, robot trajectory, and visited waypoints
% 
%   PLOTDATASTORE(DATASTORE) plots 
%   the provided map, including known walls, optional walls, beacons, 
%   waypoints, and extra credit waypoints
%
%   INPUTS  
%       dataStore   struct containing run information
%       optWalls    N-by-4 matrix containing the coordinates of optional 
%                   walls in the environment: [x1, y1, x2, y2]
%       beaconLoc   N-by-3 matrix containing the coordinates of beacons in
%                   the environment: [id, x, y]
%       waypoints   N-by-2 matrix containing the coordinates of waypoints in
%                   the environment: [x, y]
%       ECwaypoints N-by-2 matrix containing the coordinates of extra credit 
%                   waypoints in the environment: [x, y]
%
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

% Set default values if not provided
if nargin < 5
    optWalls = [];
    beaconLoc = [];
    waypoints = [];
    ECwaypoints = [];
end

% Load run information from dataStore
map = dataStore.finalMap;
pose = dataStore.robotPose;
wpts = dataStore.visitWaypoints;

% Plot final map
plotMap(map,optWalls,beaconLoc,waypoints,ECwaypoints);

% Plot robot starting point
plot(pose(1,2),pose(1,3),'bp','LineWidth',2)

% Plot robot trajectory
plot(pose(:,2),pose(:,3),'b','LineWidth',2)

% Plot waypoints
for i=1:size(wpts,1)
    plot(wpts(i,2),wpts(i,3),'rs','LineWidth',2)
end

title('Run Overview')
