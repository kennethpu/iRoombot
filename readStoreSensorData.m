function [dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,dataStore)
% This function tries to read all the sensor information from the Create
% and store it in a data structure
%
%   [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore) runs 
% 
%   INPUTS
%       CreatePort   Create port object (get from running RoombaInit)
%       SonarPort    Sonar port object (get from running RoombaInit)
%       BeaconPort   Camera port object (get from running RoombaInit)
%       dataStore    struct containing logged data
% 
%   OUTPUTS
%       dataStore   Updated struct containing logged data
% 
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

%----------------------------------------------------------
% READ ODOMETRY DISTANCE & ANGLE
%----------------------------------------------------------
% Odometry is saved in the format [toc,distance,angle]
try
    deltaD = DistanceSensorRoomba(CreatePort);
    deltaA = AngleSensorRoomba(CreatePort);
    dataStore.odometry = [dataStore.odometry; toc deltaD deltaA];
catch
    disp('Error retrieving or saving odometry data.');
end

%----------------------------------------------------------
% READ BUMP DATA
%----------------------------------------------------------
% Bump data is saved in the format [toc,right,front,left]
try
    [bumpR bumpL , ~, ~, ~, bumpF] = ...
        BumpsWheelDropsSensorsRoomba(CreatePort);
    dataStore.bump = [dataStore.bump ; toc bumpR bumpF bumpL];
catch
    disp('Error retrieving or saving bump sensor data.');
end

%----------------------------------------------------------
% READ SONAR DATA
%----------------------------------------------------------
% Sonar data is saved in the format [toc,right,front,left]
try
    if isa(SonarPort,'CreateRobot')
        % Read sonar data from Simulator
        sonarR = ReadSonar(SonarPort,1);
        sonarF = ReadSonar(SonarPort,2);
        sonarL = ReadSonar(SonarPort,3);

        % Check for empty returns (out of range readings)
        if isempty(sonarR), sonarR = NaN; end
        if isempty(sonarF), sonarF = NaN; end
        if isempty(sonarL), sonarL = NaN; end
    else
        % Read sonar data from the iRobot Create
        sonar = ReadSonar(SonarPort,[1,2,3]);
        sonarR = sonar(1); sonarF = sonar(2); sonarL = sonar(3);
    end
    dataStore.sonar = [dataStore.sonar ; toc sonarR sonarF sonarL];
catch
    disp('Error retrieving or saving sonar data.');
end

%----------------------------------------------------------
% READ CAMERA DATA (BEACONS)
%----------------------------------------------------------
% Camera data is saved in the format {toc, seen-beacons}
% seen-beacons is in the format [tag,x,y] or [] if no beacons seen
try
    if isa(BeaconPort,'CreateRobot')
        % Read AR Tags
        [Xvec,~,Zvec,~,TAGvec] = ReadBeacon(BeaconPort);
        beacons = [TAGvec,Zvec,Xvec];
    else
        % Read beacon data from the iRobot Create
        beacons = ReadBeacon(BeaconPort);
    end
    dataStore.beacon = [dataStore.beacon ; {toc beacons}];
catch
    disp('Error retrieving or saving camera data.');
end

end
