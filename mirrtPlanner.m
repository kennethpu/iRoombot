function[dataStore] = mirrtPlanner(CreatePort,SonarPort,BeaconPort,tagNum,maxTime)
% MIRRTPLANNER: Robot makes use of rapidly-exploring random trees to determine
% a collision-free path from initial position to a goal position, then moves
% along the path to the goal position
% 
%   dataStore = BIRRTPLANNER(CreatePort,SonarPort,BeaconPort,tagNum,maxTime) runs 
% 
%   INPUTS
%       CreatePort  Create port object (get from running RoombaInit)
%       SonarPort   Sonar port object (get from running RoombaInit)
%       BeaconPort  Camera port object (get from running RoombaInit)
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data
% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295) 

%==============================================================================
% SET UNSPECIFIED INPUTS
%==============================================================================
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    SonarPort = CreatePort;
    BeaconPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 3
    BeaconPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 5
    maxTime = 500;
end

%==============================================================================
% INITIALIZE VARIABLES
%==============================================================================
% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'sonar', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'finalMap', [], ...
                   'visitWaypoints', []);

% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

% Robot parameters
maxV = 0.2;         % Maximum wheel velocity
robotRad = 0.16;     % Robot radius
bumpAngles = [-pi/4, 0 pi/4];   % Bump sensor positions
epsilon = 0.12;      
closeEnough = 0.2;
fail = 0;
step = 0.5;
seed = 64;
n = 1000;

% Load map
% Exmap = load('ExampleMap1_2014.mat');
Exmap = load('ExampleMap2_2014.mat');
% Exmap = load('ExampleMap3_2014.mat');
map = Exmap.map;
optWalls = Exmap.optWalls;
beaconLoc = Exmap.beaconLoc;
waypoints = Exmap.waypoints;
ECwaypoints = Exmap.ECwaypoints;

% Plot map
plotMap(map,optWalls,beaconLoc,waypoints,ECwaypoints);

% Generate roadmap
[V,E]=treeMIRRT(map,waypoints,ECwaypoints,step,n,robotRad,seed);

%==============================================================================
% RUN CONTROL LOOP
%==============================================================================
tic

% Get initial robot position
[noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
robXY = [dataStore.truthPose(end,2) dataStore.truthPose(end,3)];

% Determine path from initial position to closest goal
[path,goal] = findPath(map,V,E,robXY,waypoints,ECwaypoints,robotRad);

gotopt = 1;         % Index for current waypoint

% % Initialize plotting of robot trajectory
% traj = plot(robXY(1),robXY(2),'r-');

while (toc < maxTime)
    while (gotopt <= size(path,1))
        % Read and store sensor data
        [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);

        % Get robot x/y position (for easier referencing)
        robXY = [dataStore.truthPose(end,2) dataStore.truthPose(end,3)];

%         % Update robot trajectory
%         tx = get(traj,'XData');
%         ty = get(traj,'YData');
%         tx = [tx robXY(1)];
%         ty = [ty robXY(2)];
%         set(traj,'XData',tx,'YData',ty);

        % Get target velocity in x and y inertial frames 
        V_x = path(gotopt,1)-robXY(1);
        V_y = path(gotopt,2)-robXY(2);

        % Get current robot orientation
        robTH = dataStore.truthPose(end,4);

        % Set angular velocity
        [fwdVel angVel] = feedbackLin(robTH, V_x, V_y, epsilon);

        % Get scaled velocity commands to avoid saturating wheels
        [cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,robotRad);

        % if overhead localization loses the robot for too long, stop it
        if noRobotCount >= 3
            SetFwdVelAngVelCreate(CreatePort, 0,0);
        else
            SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
        end
        
        % Front bump sensor triggered
        if dataStore.bump(end,7)
            bumpPt = [robXY(1)+cos(robTH+bumpAngles(2))*robotRad,robXY(2)+sin(robTH+bumpAngles(2))*robotRad];
            [wall,isOpt] = ptAtOptWall(bumpPt,map,optWalls);
            travelDist(CreatePort, maxV, -.25);            
            [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
            robXY = [dataStore.truthPose(end,2) dataStore.truthPose(end,3)];
            if (isOpt)
                fail = 1;
                [map,optWalls,V,E] = addWall(map,V,E,robotRad,optWalls,wall);
                break;
            end
            
        % Right bump sensor triggered
        elseif dataStore.bump(end,2)
            bumpPt = [robXY(1)+cos(robTH+bumpAngles(1))*robotRad,robXY(2)+sin(robTH+bumpAngles(1))*robotRad];
            [wall,isOpt] = ptAtOptWall(bumpPt,map,optWalls);
            travelDist(CreatePort, maxV, -.25);
            turnAngle(CreatePort, 0.2, 30);
            [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
            robXY = [dataStore.truthPose(end,2) dataStore.truthPose(end,3)];
            if (isOpt)
                fail = 1;
                [map,optWalls,V,E] = addWall(map,V,E,robotRad,optWalls,wall);
                break;
            end
            
        % Left bump sensor triggered
        elseif dataStore.bump(end,3)
            bumpPt = [robXY(1)+cos(robTH+bumpAngles(3))*robotRad,robXY(2)+sin(robTH+bumpAngles(3))*robotRad];
            [wall,isOpt] = ptAtOptWall(bumpPt,map,optWalls);
            travelDist(CreatePort, maxV, -.25);
            turnAngle(CreatePort, 0.2, -30);
            [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
            robXY = [dataStore.truthPose(end,2) dataStore.truthPose(end,3)];
            if (isOpt)
                fail = 1;
                [map,optWalls,V,E] = addWall(map,V,E,robotRad,optWalls,wall);
                break;
            end
        end
            
        % If robot is within closeEnough of waypoint, progress to next waypoint
        if pdist([path(gotopt,1:2);robXY],'euclidean')<=closeEnough;
            gotopt = gotopt + 1;
        end

        pause(0.1);
        end
    
    if (~fail)        
        % set forward and angular velocity to zero (stop robot)
        SetFwdVelAngVelCreate(CreatePort, 0,0 );
        
        % Set LEDs to red to indicate waypoint reached
        SetLEDsRoomba(CreatePort,1,100,100);

        % Add goal to visited waypoints
        dataStore.visitWaypoints = [dataStore.visitWaypoints;toc,goal];
        
        % Remove goal from list of waypoints
        [inWaypts,idx] = ismember(goal,waypoints,'rows');
        if(inWaypts) 
            waypoints(idx,:) = [];
        else
            [~,idx] = ismember(goal,ECwaypoints,'rows');
            ECwaypoints(idx,:) = [];
        end

        if ((size(waypoints,1)+size(ECwaypoints,1))==0)
            break;
        end
    end
    
    % Determine path from current position to new goal
    [path,goal,gfound] = findPath(map,V,E,robXY,waypoints,ECwaypoints,robotRad);
    
    if (gfound == 0) 
        [V,E]=treeMIRRT(map,V,[],step,n,robotRad,0);
        [path,goal,gfound] = findPath(map,V,E,robXY,waypoints,ECwaypoints,robotRad);
    end
        
    gotopt = 1;         % Index for which current waypoint
    fail = 0;
    
    % Set LEDs to green to indicate leaving waypoint
    SetLEDsRoomba(CreatePort,1,0,100);
    
end

% Store final map in dataStore
dataStore.finalMap = map;

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
