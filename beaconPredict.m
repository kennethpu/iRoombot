function [beacons] = beaconPredict(robotPose,map,beaconLoc,robotRad,angRange,depthRange)
% BEACONPREDICT: predict the beacon position measurements for a robot operating
% in a known map. Beacon positions will be relative to the robot
% 
%   BEACONS = BEACONPREDICT(ROBOTPOSE,MAP,ROBOTRAD,ANGLES,MAXRANGE) returns 
%   the expected beacon position measurements for a robot operating
%   in a known map. Beacon positions will be relative to the robot
% 
%   INPUTS
%       robotPose   1-by-3 pose vector in global coordinates [x,y,theta]
%       map         N-by-4 matrix containing the coordinates of walls in 
%                   the environment: [x1, y1, x2, y2]
%       optWalls    M-by-4 matrix containing the coordinates of optional 
%                   walls in the environment: [x1, y1, x2, y2]
%       beaconLoc   N-by-3 matrix containing the coordinates of beacons in
%                   the environment: [id, x, y]
%       robotRad    robot radius (meters)
%       angRange    maximum camera angular range
%       depthRange  maximum camera depth range (meters) 
% 
%   OUTPUTS
%       beacons     K-by-3 matrix containing beacon position measurements
% 
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

% TODO: ADD OPTWALLS

%% ============================================================================
% INITIALIZE VARIABLES
%==============================================================================
% Extract robot x,y and theta to individual variables
rob_x = robotPose(1);
rob_y = robotPose(2);
rob_theta = robotPose(3);

% REMOVE: Plot robot as a circle of the correct radius, with a line to designate
% orientation
% ang = 0:0.01:2*pi;
% plot(rob_x+robotRad*cos(ang), rob_y+robotRad*sin(ang));
% plot([rob_x rob_x+robotRad*cos(rob_theta)], [rob_y rob_y+robotRad*sin(rob_theta)]);

% Get polygon representing camera viewable range
polyC = getPolygon(rob_x,rob_y,rob_theta,robotRad,angRange,depthRange);
c_pos = [rob_x+cos(rob_theta)*robotRad,rob_y+sin(rob_theta)*robotRad];

% REMOVE: Plot camera polygon
% for i=1:size(polyC,1)
%     curPt = polyC(i,:);
%     nexPt = polyC(mod(i,size(polyC,1))+1,:);
%     plot([curPt(1) nexPt(1)],[curPt(2) nexPt(2)],'g');
% end

beacons = [];

% Determine which beacons are currently visible from robot
for i=1:size(beaconLoc,1)
    beacon = beaconLoc(i,:);
    % Check if beacon is contained in camera polygon
    if (inpolygon(beacon(2),beacon(3),polyC(:,1)',polyC(:,2)'))
        % REMOVE: Plot candidate beacons
%         plot(beacon(2),beacon(3),'rs')
        if (segIsFree([c_pos,beacon(2:3)],map))
            % Add beacon to output array
%             beacons = [beacons; beacon(1),global2robot([c_pos,rob_theta],beacon(2:3))];
            beacons = [beacons; beacon(1),global2robot(robotPose,beacon(2:3))];
            % REMOVE: Plot valid line of sight to beacons
%             plot([c_pos(1),beacon(2)],[c_pos(2),beacon(3)],'r')
        end
    end
end
end

%% ============================================================================
% getPolygon
%==============================================================================
%   Helper function to get polygons corresponding to the cone perceived by
%   forward facing camera
%
%   INPUTS 
%       rx          robot x coordinate
%       ry          robot y coordinate
%       rt          robot orientation
%       rr          robot radius
%       angRange    maximum camera angular range
%       depthRange  maximum camera depth range (meters) 
% 
%   OUTPUTS 
%       PolyC       polygon corresponding to front camera
function [polyC] = getPolygon(rx,ry,rt,rr,angRange,depthRange)
    
    % Construct polygon corresponding to front sonar sensor
    % Get vertices that compose front camera polygon
    angF = rt;
    angFr = angF-angRange/2;
    angFl = angF+angRange/2;
    rn = rr+depthRange;
    polyC = [rx+cos(angFr)*rr ry+sin(angFr)*rr; ...
             rx+cos(angF)*rr  ry+sin(angF)*rr;  ...
             rx+cos(angFl)*rr ry+sin(angFl)*rr; ...
             rx+cos(angFl)*rn ry+sin(angFl)*rn; ...
             rx+cos(angF)*rn  ry+sin(angF)*rn; ...
             rx+cos(angFr)*rn ry+sin(angFr)*rn; ...
             rx+cos(angFr)*rr ry+sin(angFr)*rr];
end