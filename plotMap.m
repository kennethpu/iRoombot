function plotMap(map,optWalls,beaconLoc,waypoints,ECwaypoints)
% PLOTMAP: Plots the provided map, including known walls, optional walls,
% beacons, waypoints, and extra credit waypoints
% 
%   PLOTMAP(MAP) plots 
%   the provided map, including known walls, optional walls, beacons, 
%   waypoints, and extra credit waypoints
%
%   INPUTS  
%       map         N-by-4 matrix containing the coordinates of walls in the 
%                   environment: [x1, y1, x2, y2]
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
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

%% ============================================================================
% INITIALIZE VARIABLES
%==============================================================================
x_min = min(min(map(:,1)),min(map(:,3)));
x_max = max(max(map(:,1)),max(map(:,3)));
y_min = min(min(map(:,2)),min(map(:,4)));
y_max = max(max(map(:,2)),max(map(:,4)));

%% ============================================================================
% SET UP NEW FIGURE
%==============================================================================
figure;
hold on;
% Set axis to just fit map
axis([x_min x_max y_min y_max]);
axis equal;
axis manual;
xlabel('x (meters)')
ylabel('y (meters)')
title('Map')

%% ============================================================================
% PLOT MAP
%==============================================================================
% Plot walls
for i=1:size(map,1)
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'k','LineWidth',3);
end

% Plot optional walls
for i=1:size(optWalls,1)
    plot([optWalls(i,1) optWalls(i,3)],[optWalls(i,2) optWalls(i,4)],'m--','LineWidth',3);
end

% Plot beacons
for i=1:size(beaconLoc,1)
    plot(beaconLoc(i,2),beaconLoc(i,3),'bo','LineWidth',2);
    text(beaconLoc(i,2),beaconLoc(i,3)+.15,int2str(beaconLoc(i,1)));
end

% Plot waypoints
for i=1:size(waypoints,1)
    plot(waypoints(i,1),waypoints(i,2),'gs','LineWidth',2);
end

% Plot extra credit waypoints
for i=1:size(ECwaypoints,1)
    plot(ECwaypoints(i,1),ECwaypoints(i,2),'gp','LineWidth',2);
end
