function [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore,BeaconMode)
% This function tries to read all the sensor information from the Create
% and store it in a data structure
%
%   [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore) runs 
% 
%   INPUTS
%       CreatePort   Create port object (get from running RoombaInit)
%       SonarPort    Sonar port object (get from running RoombaInit)
%       BeaconPort   Camera port object (get from running RoombaInit)
%       tagNum       robot number for overhead localization
%       noRobotCount Number of consecutive times the robot was "lost" by the overhead localization
%       dataStore    struct containing logged data
%       BeaconMode   1:(DEFAULT) AR Tag Mode [time, tag number, X_camera, Y_camera, Z_camera, rot_camera]
%                    0: Blob Detection Mode [time, [3 element color vector], bearing, range] 
% 
%   OUTPUTS
%       noRobotCount   Updated number of consecutive times the robot was "lost" by the overhead localization
%       dataStore   Updated struct containing logged data
%
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots

% If optional input 'BeaconMode' was not provided.  Assign default value
if nargin<7, BeaconMode = 1; end

    % read truth pose (from overhead localization system)
    try
        [px, py, pt] = OverheadLocalizationCreate(tagNum);
        if (px == 0 && py == 0 && pt == 0 && ~isa(tagNum,'CreateRobot'))
            disp('Overhead localization lost the robot!')
            noRobotCount = noRobotCount + 1;
        else
            poseX = px; poseY = py; poseTheta = pt;
            dataStore.truthPose = [dataStore.truthPose ; ...
                               toc poseX poseY poseTheta];
            noRobotCount = 0;
        end
    catch
        disp('Error retrieving or saving overhead localization data.');
    end
    
    % read odometry distance & angle
    try
        deltaD = DistanceSensorRoomba(CreatePort);
        deltaA = AngleSensorRoomba(CreatePort);
        dataStore.odometry = [dataStore.odometry ; ...
                              toc deltaD deltaA];
    catch
        disp('Error retrieving or saving odometry data.');
    end
    
%     % read lidar data
%     try
%         if isa(CreatePort,'CreateRobot')
%             % Read lidar data from Simulator
%             lidarScan = LidarSensorCreate(CreatePort);
%             dataStore.lidar = [dataStore.lidar ; toc lidarScan];
%         else
%             % Lidar data is not available from the iRobot Create
%         end
%     catch
%         disp('Error retrieving or saving lidar data.');
%     end
    
    % read bump data
    try
        [BumpRight BumpLeft DropRight DropLeft DropCaster BumpFront] = ...
            BumpsWheelDropsSensorsRoomba(CreatePort);
        dataStore.bump = [dataStore.bump ; toc ...
            BumpRight BumpLeft DropRight DropLeft DropCaster BumpFront];
    catch
        disp('Error retrieving or saving bump sensor data.');
    end
    
    % read sonar data
    try
        if isa(SonarPort,'CreateRobot')
            Read sonar data from Simulator
            sonarR = ReadSonar(SonarPort,1);
            sonarF = ReadSonar(SonarPort,2);
            sonarL = ReadSonar(SonarPort,3);
            
            Check for empty returns (out of range readings)
            if isempty(sonarR), sonarR = NaN; end
            if isempty(sonarF), sonarF = NaN; end
            if isempty(sonarL), sonarL = NaN; end
        else
            Read sonar data from the iRobot Create
            sonar = ReadSonar(SonarPort,[1,2,3]);
            sonarR = sonar(1); sonarF = sonar(2); sonarL = sonar(3);
        end
        dataStore.sonar = [dataStore.sonar ; toc sonarR sonarF sonarL];
    catch
        disp('Error retrieving or saving sonar data.');
    end

    % read camera data (beacons)
    try
        if isa(BeaconPort,'CreateRobot')
            switch BeaconMode
                case 0
                    % Use blob detection
                    [angle dist color]= CameraSensorCreate(BeaconPort);
                    beacons = [color, angle, dist];
                case 1
                    % Read AR Tags
                    [Xvec,Yvec,Zvec,ROTvec,TAGvec] = ReadBeacon(BeaconPort);
                    beacons = [TAGvec,Xvec,Yvec,Zvec,ROTvec];
            end
        else
            % Read beacon data from the iRobot Create
            beacons = ReadBeacon(BeaconPort);
        end
        if ~isempty(beacons)
            dataStore.beacon = [dataStore.beacon ; repmat(toc,size(beacons,1),1) beacons];
        end
    catch
        disp('Error retrieving or saving camera data.');
    end
