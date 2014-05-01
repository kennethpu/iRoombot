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

% REMOVE:
fprintf('Starting...\n');

%% ============================================================================
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

%% ============================================================================
% INITIALIZE VARIABLES
%==============================================================================
% Declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% Initialize datalog struct (customize according to needs)
% REMOVE: Currently uses truthPose
dataStore = struct('robotPose', [], ...
                   'odometry', [], ...
                   'particles', [], ...
                   'sonar', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'finalMap', [], ...
                   'visitWaypoints', []);

% Initialize robot parameters
maxV = 0.2;                     % Maximum wheel velocity
robotRad = 0.16;                % Robot radius
wheelRad = 0.129;               % Wheel radius
bumpAngles = [-pi/4, 0 pi/4];   % Bump sensor positions  
sonarAngles = [-pi/2 0 pi/2];   % Sonar sensor positions
maxDepth = 20;                  % Maximum depth range of camera and sonar sensors used for measurement predictions (deliberately set high so we don't throw out data even if the physical sensors do)
angRangeC = 2*pi/3;             % Angular range of camera sensor
epsilon = 0.2;                  % "Curvature" of robot turning
closeEnough = 0.2;              % Minimum distance from waypoint before it counts as "visited"

% Initialize roadmap parameters
radExt = 0.04;  % How much to overestimate robot radius 
step = 0.5;     % Step size
seed = 81; %169;     % Number of seed nodes to sample 
n = 1000;       % Stopping criteria

% Initialize localization parameters
R = [.001,    0, 0;
        0, .001, 0;
        0,    0, .01];   % Process Noise
Qs = 0.01 * eye(3);      % Sonar Measurement Noise
Qb = 0.0001 * eye(3);    % Beacon Measurement Noise
N = 15;                  % Number of particles (per waypoint)

% Initialize control function flags
fail = 0;       % Flag indicating whether goal was found

%% ============================================================================
% LOAD MAP PARAMETERS
%==============================================================================
% Load .mat file
Exmap = load('ExampleMap1_2014.mat');
% Exmap = load('ExampleMap2_2014.mat');
% Exmap = load('ExampleMap3_2014.mat');

% Load map parameters from .mat file
map = Exmap.map;                    % Known walls in environment [Nx4]
optWalls = Exmap.optWalls;          % Optional walls in environment [Mx4]
waypoints = Exmap.waypoints;        % x/y positions of waypoints [Kx2]
ECwaypoints = Exmap.ECwaypoints;    % x/y positions of extra credit waypoints [Jx2]
beaconLoc = Exmap.beaconLoc;        % tag and x/y positions of beacons [Lx3]

%% ============================================================================
% ADDITIONAL PRE-PROCESSING STEPS
%==============================================================================
% REMOVE: Plot map
plotMap(map,optWalls,beaconLoc,waypoints,ECwaypoints);

% Generate roadmap
[V,E]=treeMIRRT(map,waypoints,ECwaypoints,step,n,robotRad+radExt,seed);

% Initialize particle set (N particles for each waypoint)
p_init = zeros(N*size(waypoints,1),4);
for i = 1:size(waypoints,1) 
    % XY coordinates are normally distributed around waypoint
    randXY = mvnrnd(repmat(waypoints(i,1:2),N,1),.01*eye(2));
    % Theta is uniformly distributed over -pi to pi
    randT = [-pi:2*pi/(N-1):pi]';
    p_init((i-1)*N+1:i*N,:) = [randXY,randT,repmat([1/(N*size(waypoints,1))],N,1)];
end
dataStore.particles = [dataStore.particles; {toc,p_init}];

%---------------------------------------------------------------
% REMOVE: INITIALIZE PLOT HANDLES
%---------------------------------------------------------------
parts_p = plot(p_init(:,1),p_init(:,2),'ro');   % Particle set
wpts_p = plot(10,10,'go','LineWidth',3);        % Path waypoints (nodes)
path_p = plot(10,10,'g','LineWidth',3);         % Path edges
goal_p = plot(10,10,'rs','LineWidth',3);        % Goal point 
traj_p = plot(10,10,'rv','LineWidth',2);        % Robot position

%% ============================================================================
% RUN CONTROL LOOP
%==============================================================================
% Start timer
tic

%---------------------------------------------------------------
% LOCALIZATION LOOP
%---------------------------------------------------------------
% Turn in place for a short while to allow robot to localize
while toc < 2.5
    % Read and store sensor data
    [dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,dataStore);
    
    % Update particle set
    Pset_i = cell2mat(dataStore.particles(end,2));
    d = dataStore.odometry(end,2);
    phi = dataStore.odometry(end,3);
    
%     z = cell2mat(dataStore.beacon(end,2))
    z = dataStore.sonar(end,2:end);
    g_fun = @(pose)integrateOdom(pose,d,phi);
    h_fun = @(pose,map)sonarPredict(pose,map,robotRad,sonarAngles,maxDepth);
    Pset_f = particleFilter(Pset_i,map,robotRad,z,g_fun,h_fun,R,Qs);
    dataStore.particles = [dataStore.particles; {toc Pset_f}];
    
    % REMOVE: Plot updated particle set
    psetp = Pset_f(Pset_f(:,4)>0,:);
    set(parts_p,'XData',psetp(:,1),'YData',psetp(:,2));
    
%     for p = 1:size(p_init,1)
%         x = Pset_f(p,1);
%         y = Pset_f(p,2);
%         theta = Pset_f(p,3);
%         plot([x x+.3*cos(theta)],[y y+.3*sin(theta)],'r');
%     end
    
    [cmdV,cmdW] = limitCmds(0,-1,maxV,wheelRad);
    SetFwdVelAngVelCreate(CreatePort,cmdV,cmdW);
    pause(0.1);
end    

%---------------------------------------------------------------
% LOCALIZATION LOOP ENDED, GUESS INITIAL POSITION
%---------------------------------------------------------------
% Stop robot first
SetFwdVelAngVelCreate(CreatePort,0,0);
% Use simple average of particles to determine robot position
robXY = [sum(Pset_f(:,1))/size(Pset_f,1),sum(Pset_f(:,2))/size(Pset_f,1)];
robTH = [sum(Pset_f(:,3))/size(Pset_f,1)];
dataStore.robotPose = [dataStore.robotPose;toc,robXY,robTH];

%---------------------------------------------------------------
% FIND PATH FROM INITIAL POSITION TO CLOSEST GOAL
%---------------------------------------------------------------
[path,goal,~] = findPath(map,V,E,robXY,waypoints,ECwaypoints,robotRad+radExt);
gotopt = 1;         % Index for current waypoint
set(wpts_p,'XData',path(:,1),'YData',path(:,2));
set(path_p,'XData',path(:,1),'YData',path(:,2));
set(goal_p,'XData',goal(1),'YData',goal(2));

% REMOVE:
fprintf('New goal point [%f,%f]\n',goal(1),goal(2));

% REMOVE: Update robot trajectory plot
set(traj_p,'XData',robXY(1),'YData',robXY(2));

% REMOVE:
fprintf('Next waypoint [%f,%f]\n',path(gotopt,1),path(gotopt,2));

%---------------------------------------------------------------
% RUN CONTROL LOOP UNTIL TIMEOUT (OR ALL WAYPOINTS VISITED)
%---------------------------------------------------------------
while (toc < maxTime)
    %---------------------------------------------------------------
    % VISIT ALL WAYPOINTS IN CURRENT PATH
    %---------------------------------------------------------------
    while ((gotopt <= size(path,1)) && (toc < maxTime))
        % Read and store sensor data
        [dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,dataStore);
      
        % Update particle set
        Pset_i = cell2mat(dataStore.particles(end,2));
        z = dataStore.sonar(end,2:end);
        d = dataStore.odometry(end,2);
        phi = dataStore.odometry(end,3);
        g_fun = @(pose)integrateOdom(pose,d,phi);
        h_fun = @(pose,map)sonarPredict(pose,map,robotRad,sonarAngles,maxDepth);
        Pset_f = particleFilter(Pset_i,map,robotRad,z,g_fun,h_fun,R,Qs);
        dataStore.particles = [dataStore.particles; {toc Pset_f}];
        
        % REMOVE: Plot updated particle set
        psetp = Pset_f(Pset_f(:,4)>0,:);
        set(parts_p,'XData',psetp(:,1),'YData',psetp(:,2));
        
        % Estimate current robot pose using simple average of particles
        robXY = [sum(Pset_f(:,1))/size(Pset_f,1),sum(Pset_f(:,2))/size(Pset_f,1)];
        robTH = sum(Pset_f(:,3))/size(Pset_f,1);
        dataStore.robotPose = [dataStore.robotPose;toc,robXY,robTH];

        % REMOVE: Update robot trajectory plot 
%         tx = get(traj,'XData');
%         ty = get(traj,'YData');
%         tx = [tx robXY(1)];
%         ty = [ty robXY(2)];
        set(traj_p,'XData',robXY(1),'YData',robXY(2));

        %---------------------------------------------------------------
        % DETERMINE WHEEL COMMANDS
        %---------------------------------------------------------------
        % Get target velocity in x and y inertial frames 
        V_x = path(gotopt,1)-robXY(1);
        V_y = path(gotopt,2)-robXY(2);

        % Set angular velocity
        [fwdVel angVel] = feedbackLin(robTH, V_x, V_y, epsilon);

        % Get scaled velocity commands to avoid saturating wheels
        [cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheelRad);
        
        % REMOVE:
%         fprintf('[%d](%f,%f), fwdVel:%f, angVel:%f, cmdV:%f, cmdW:%f\n',size(dataStore.truthPose,1),robXY(1),robXY(2),fwdVel,angVel,cmdV,cmdW);
        
        % Send wheel commands to robot
        SetFwdVelAngVelCreate(CreatePort,cmdV,cmdW);
        
        %---------------------------------------------------------------
        % HANDLE BUMPS
        %---------------------------------------------------------------
        % Front bump sensor triggered
        if dataStore.bump(end,3)
            % Determine bump location
            bumpPt = [robXY(1)+cos(robTH+bumpAngles(2))*robotRad,robXY(2)+sin(robTH+bumpAngles(2))*robotRad];
            % Check if bump location corresponds to an optional wall
            [wall,isOpt] = ptAtOptWall(bumpPt,map,optWalls);
            % Back up robot
            travelDist(CreatePort, maxV, -.25);  
            
            % Read and store sensor data
            [dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,dataStore);

            % Update particle set
            Pset_i = cell2mat(dataStore.particles(end,2));
            z = dataStore.sonar(end,2:end);
            d = dataStore.odometry(end,2);
            phi = dataStore.odometry(end,3);
            g_fun = @(pose)integrateOdom(pose,d,phi);
            h_fun = @(pose,map)sonarPredict(pose,map,robotRad,sonarAngles,maxDepth);
            Pset_f = particleFilter(Pset_i,map,robotRad,z,g_fun,h_fun,R,Qs);
            dataStore.particles = [dataStore.particles; {toc Pset_f}];

            % REMOVE: Update robot trajectory plot 
            psetp = Pset_f(Pset_f(:,4)>0,:);
            set(parts_p,'XData',psetp(:,1),'YData',psetp(:,2));
            
            % Estimate current robot pose using simple average of particles
            robXY = [sum(Pset_f(:,1))/size(Pset_f,1),sum(Pset_f(:,2))/size(Pset_f,1)];
            robTH = sum(Pset_f(:,3))/size(Pset_f,1);
            dataStore.robotPose = [dataStore.robotPose;toc,robXY,robTH];

            % If robot has bumped into an optional wall, add the wall to
            % map, update roadmap accordingly, and break out of loop with
            % fail flag raised
            if (isOpt)
                fail = 1;
                [map,optWalls,V,E] = addWall(map,V,E,robotRad+radExt,optWalls,wall);
                dataStore.finalMap = map;
                break;
            end
            
        % Right bump sensor triggered
        elseif dataStore.bump(end,2)
            % Determine bump location
            bumpPt = [robXY(1)+cos(robTH+bumpAngles(1))*robotRad,robXY(2)+sin(robTH+bumpAngles(1))*robotRad];
            % Check if bump location corresponds to an optional wall
            [wall,isOpt] = ptAtOptWall(bumpPt,map,optWalls);
            % Back up robot
            travelDist(CreatePort, maxV, -.25);
            
            % Read and store sensor data
            [dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,dataStore);

            % Update particle set
            Pset_i = cell2mat(dataStore.particles(end,2));
            z = dataStore.sonar(end,2:end);
            d = dataStore.odometry(end,2);
            phi = dataStore.odometry(end,3);
            g_fun = @(pose)integrateOdom(pose,d,phi);
            h_fun = @(pose,map)sonarPredict(pose,map,robotRad,sonarAngles,maxDepth);
            Pset_f = particleFilter(Pset_i,map,robotRad,z,g_fun,h_fun,R,Qs);
            dataStore.particles = [dataStore.particles; {toc Pset_f}];

            % REMOVE: Plot updated particle set
            psetp = Pset_f(Pset_f(:,4)>0,:);
            set(parts_p,'XData',psetp(:,1),'YData',psetp(:,2));
            
            % Estimate current robot pose using simple average of particles
            robXY = [sum(Pset_f(:,1))/size(Pset_f,1),sum(Pset_f(:,2))/size(Pset_f,1)];
            robTH = sum(Pset_f(:,3))/size(Pset_f,1);
            dataStore.robotPose = [dataStore.robotPose;toc,robXY,robTH];
            
            % Turn robot 30 degrees counterclockwise
            turnAngle(CreatePort, maxV, 30); 
            
            % Read and store sensor data
            [dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,dataStore);

            % Update particle set
            Pset_i = cell2mat(dataStore.particles(end,2));
            z = dataStore.sonar(end,2:end);
            d = dataStore.odometry(end,2);
            phi = dataStore.odometry(end,3);
            g_fun = @(pose)integrateOdom(pose,d,phi);
            h_fun = @(pose,map)sonarPredict(pose,map,robotRad,sonarAngles,maxDepth);
            Pset_f = particleFilter(Pset_i,map,robotRad,z,g_fun,h_fun,R,Qs);
            dataStore.particles = [dataStore.particles; {toc Pset_f}];

            % REMOVE: Plot updated particle set
            psetp = Pset_f(Pset_f(:,4)>0,:);
            set(parts_p,'XData',psetp(:,1),'YData',psetp(:,2));
            
            % Estimate current robot pose using simple average of particles
            robXY = [sum(Pset_f(:,1))/size(Pset_f,1),sum(Pset_f(:,2))/size(Pset_f,1)];
            robTH = sum(Pset_f(:,3))/size(Pset_f,1);
            dataStore.robotPose = [dataStore.robotPose;toc,robXY,robTH];

            % If robot has bumped into an optional wall, add the wall to
            % map, update roadmap accordingly, and break out of loop with
            % fail flag raised
            if (isOpt)
                fail = 1;
                [map,optWalls,V,E] = addWall(map,V,E,robotRad+radExt,optWalls,wall);
                dataStore.finalMap = map;
                break;
            end
            
        % Left bump sensor triggered
        elseif dataStore.bump(end,4)
            % Determine bump location
            bumpPt = [robXY(1)+cos(robTH+bumpAngles(3))*robotRad,robXY(2)+sin(robTH+bumpAngles(3))*robotRad];
            % Check if bump location corresponds to an optional wall
            [wall,isOpt] = ptAtOptWall(bumpPt,map,optWalls);
            % Back up robot
            travelDist(CreatePort, maxV, -.25);
            
            % Read and store sensor data
            [dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,dataStore);

            % Update particle set
            Pset_i = cell2mat(dataStore.particles(end,2));
            z = dataStore.sonar(end,2:end);
            d = dataStore.odometry(end,2);
            phi = dataStore.odometry(end,3);
            g_fun = @(pose)integrateOdom(pose,d,phi);
            h_fun = @(pose,map)sonarPredict(pose,map,robotRad,sonarAngles,maxDepth);
            Pset_f = particleFilter(Pset_i,map,robotRad,z,g_fun,h_fun,R,Qs);
            dataStore.particles = [dataStore.particles; {toc Pset_f}];

            % REMOVE: Plot updated particle set
            psetp = Pset_f(Pset_f(:,4)>0,:);
            set(parts_p,'XData',psetp(:,1),'YData',psetp(:,2));
            
            % Estimate current robot pose using simple average of particles
            robXY = [sum(Pset_f(:,1))/size(Pset_f,1),sum(Pset_f(:,2))/size(Pset_f,1)];
            robTH = sum(Pset_f(:,3))/size(Pset_f,1);
            dataStore.robotPose = [dataStore.robotPose;toc,robXY,robTH];
            
            % Turn 30 degrees clockwise
            turnAngle(CreatePort, maxV, -30);
            
            % Read and store sensor data
            [dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,dataStore);

            % Update particle set
            Pset_i = cell2mat(dataStore.particles(end,2));
            z = dataStore.sonar(end,2:end);
            d = dataStore.odometry(end,2);
            phi = dataStore.odometry(end,3);
            g_fun = @(pose)integrateOdom(pose,d,phi);
            h_fun = @(pose,map)sonarPredict(pose,map,robotRad,sonarAngles,maxDepth);
            Pset_f = particleFilter(Pset_i,map,robotRad,z,g_fun,h_fun,R,Qs);
            dataStore.particles = [dataStore.particles; {toc Pset_f}];

            % REMOVE: Plot updated particle set
            psetp = Pset_f(Pset_f(:,4)>0,:);
            set(parts_p,'XData',psetp(:,1),'YData',psetp(:,2));
            
            % Estimate current robot pose using simple average of particles
            robXY = [sum(Pset_f(:,1))/size(Pset_f,1),sum(Pset_f(:,2))/size(Pset_f,1)];
            robTH = sum(Pset_f(:,3))/size(Pset_f,1);
            dataStore.robotPose = [dataStore.robotPose;toc,robXY,robTH];

            % If robot has bumped into an optional wall, add the wall to
            % map, update roadmap accordingly, and break out of loop with
            % fail flag raised
            if (isOpt)
                fail = 1;
                [map,optWalls,V,E] = addWall(map,V,E,robotRad+radExt,optWalls,wall);
                dataStore.finalMap = map;
                break;
            end
        end
            
        %---------------------------------------------------------------
        % CHECK IF CURRENTLY AT A WAYPOINT
        %---------------------------------------------------------------
        % If robot is within closeEnough of waypoint, progress to next waypoint
        if pdist([path(gotopt,1:2);robXY],'euclidean')<=closeEnough;
            fprintf('Robot reached waypoint [%f,%f]\n',path(gotopt,1),path(gotopt,2));
            gotopt = gotopt + 1;
            % REMOVE:
            if gotopt <= size(path,1)
                fprintf('Next waypoint [%f,%f]\n',path(gotopt,1),path(gotopt,2));
            end
        end
        pause(0.1);
    end

    %---------------------------------------------------------------
    % CHECK IF CURRENTLY AT A GOAL POINT
    %---------------------------------------------------------------
    if (~fail)        
        fprintf('Robot reached goal point [%f,%f]\n',goal(1),goal(2));
        % Set forward and angular velocity to zero (stop robot)
        SetFwdVelAngVelCreate(CreatePort,0,0 );
        
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
        
        % Check if all goal points have been found
        if ((size(waypoints,1)+size(ECwaypoints,1))==0)
            % REMOVE:
            fprintf('Found all goal points, exiting..\n');
            break;
        end
    end
    
    % Determine path from current position to new goal
    [path,goal,gfound] = findPath(map,V,E,robXY,waypoints,ECwaypoints,robotRad+radExt);
    
    % REMOVE: Update path and goal plots
    set(wpts_p,'XData',path(:,1),'YData',path(:,2));
    set(path_p,'XData',path(:,1),'YData',path(:,2));
    set(goal_p,'XData',goal(1),'YData',goal(2));
    
    % REMOVE:
    fprintf('New goal point [%f,%f]\n',goal(1),goal(2));
    
    % If no goals are reachable from current tree, grow trees until they
    % are connected
    if (gfound == 0) 
        [V,E]=treeMIRRT(map,V,[],step,n,robotRad+radExt,0);
        [path,goal,~] = findPath(map,V,E,robXY,waypoints,ECwaypoints,robotRad+radExt);
        
        % REMOVE: Update path and goal plots
        set(wpts_p,'XData',path(:,1),'YData',path(:,2));
        set(path_p,'XData',path(:,1),'YData',path(:,2));
        set(goal_p,'XData',goal(1),'YData',goal(2));
        
        % REMOVE:
        fprintf('New goal point [%f,%f]\n',goal(1),goal(2));
    end
        
    gotopt = 1;         % Index for which current waypoint
    fail = 0;
    
    % Set LEDs to green to indicate leaving waypoint
    SetLEDsRoomba(CreatePort,1,0,100);
    
end

% REMOVE:
fprintf('Exiting control loop\n');

% Store final map in dataStore
dataStore.finalMap = map;

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort,0,0);

% Plot run information
plotDataStore(dataStore,optWalls,beaconLoc,waypoints,ECwaypoints);

end

%% ============================================================================
% localize
%==============================================================================
%   Helper function to get the index of the node with smallest cost in a 
%   list of nodes
%
%   INPUTS 
%       Q      list of nodes to search over
%       V      list of nodes in graph
%       V_cost list of costs associated with V
% 
%   OUTPUTS 
%       nidx   index of node in Q with smallest cost
% function
