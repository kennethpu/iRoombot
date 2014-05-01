function[ranges] = sonarPredict(robotPose,map,optWalls,robotRad,angles,maxRange)
% SONARPREDICT: predict the sonar range measurements for a robot operating
% in a known map.
% 
%   RANGES = SONARPREDICT(ROBOTPOSE,MAP,ROBOTRAD,ANGLES,MAXRANGE) returns 
%   the expected sonar range measurements for a robot operating in a known 
%   map. Range measurements will be in the range [0,maxRange] or maxRange if
%   predicted range measurement is greater than maxRange.
% 
%   INPUTS
%       robotPose   1-by-3 pose vector in global coordinates [x,y,theta]
%       map         N-by-4 matrix containing the coordinates of walls in 
%                   the environment: [x1, y1, x2, y2]
%       optWalls    N-by-4 matrix containing the coordinates of optional 
%                   walls in the environment: [x1, y1, x2, y2]
%       robotRad    robot radius (meters)
%       angles      1-by-K vector of the angular positions of the sonar
%                   sensor(s) in robot coordinates, where 0 points forward
%       maxRange    maximum sonar range (meters) 
% 
%   OUTPUTS
%       ranges      1-by-K vector of sonar ranges (meters)
% 
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

%% ============================================================================
% INITIALIZE VARIABLES
%==============================================================================
% Extract robot x,y and theta to individual variables
rob_x = robotPose(1);
rob_y = robotPose(2);
rob_theta = robotPose(3);

% Initialize output ranges to max range
ranges = ones(size(angles)).*maxRange;
    
% REMOVE: Plot robot as a circle of the correct radius, with a line to designate
% orientation
% ang = 0:0.01:2*pi;
% plot(rob_x+robotRad*cos(ang), rob_y+robotRad*sin(ang));
% plot([rob_x rob_x+robotRad*cos(rob_theta)], [rob_y rob_y+robotRad*sin(rob_theta)]);

%% ============================================================================
% DETERMINE SONAR RESPONSE FOR EACH SENSOR
%==============================================================================
% Iterate through each sonar sensor, in the corresponding range array fill in 
% the distance to the closest wall or NaN if there is no wall within
% maxRange of the sensor
for sensor = 1:size(angles',1)
    % Determine endpoints for a line of length maxRange extending from the
    % robot in the same orientation as the sensor
    sens_x1 = rob_x+robotRad*cos(rob_theta+angles(sensor));
    sens_y1 = rob_y+robotRad*sin(rob_theta+angles(sensor));
    sens_x2 = rob_x+(robotRad+maxRange)*cos(rob_theta+angles(sensor));
    sens_y2 = rob_y+(robotRad+maxRange)*sin(rob_theta+angles(sensor));
    
    % REMOVE: Plot resultant sensor line
%     plot([sens_x1, sens_x2], [sens_y1, sens_y2],'g');
    
    % Iterate throuch each wall, chcking for collisions between previously
    % created sensor line and each wall. An intersection means that there
    % will be a response from the sonar
    for i = 1:size(map,1)
        % Check for collision between wall and sensor line
        [isect,x,y,ua] = intersectPoint(sens_x1,sens_y1,sens_x2,sens_y2,map(i,1),map(i,2),map(i,3),map(i,4));
        
        % If an intersection is detected update range measurements
        if(isect)
            % REMOVE: Plot intersection point if enabled
%             plot(x,y,'ro');
            
            % If intersection point is closer than previous minimum
            % distance, update variables accordingly
            distance = ua*maxRange;
            if (distance < ranges(sensor))
                ranges(sensor) = distance;
            end
        end
    end
    
    % Initialize minimum distance and collided wall variables
    optRanges = [];
    
    % Iterate throuch each optional wall, chcking for collisions between 
    % previously created sensor line and each wall. An intersection means 
    % that there MAY be a response from the sonar
    for i = 1:size(optWalls,1)
        % Check for collision between optional wall and sensor line
        [isect,x,y,ua] = intersectPoint(sens_x1,sens_y1,sens_x2,sens_y2,optWalls(i,1),optWalls(i,2),optWalls(i,3),optWalls(i,4));
        
        % If an intersection is detected update range measurements
        if(isect)
            % REMOVE: Plot intersection point if enabled
%             plot(x,y,'bo');
            
            % If intersection point is closer than previous minimum
            % distance, store distance in array
            distance = ua*maxRange;
            if (distance < ranges(sensor))
                optRanges = [optRanges;distance];
            end
        end
    end
    
    % If optional walls are closer to robot than known wall, sum their
    % weighted potential contributions and average the result
    if ~isempty(optRanges)
        optRanges = sortrows(optRanges);
        n = size(optRanges,1);
        for i=1:n
            ranges(sensor) = (ranges(sensor))+(2^(n-i))*optRanges(i);
        end
        % Save sensor value as negative to indicate presence of optional
        % wall
        ranges(sensor) = -ranges(sensor)/(2^n);
    end
    
    % REMOVE: Plot predicted range measurement
%     plot(rob_x+ranges(sensor)*cos(rob_theta+angles(sensor)),rob_y+ranges(sensor)*sin(rob_theta+angles(sensor)),'rs')
        
end

end