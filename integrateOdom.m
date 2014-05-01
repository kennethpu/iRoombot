function[pose_f] = integrateOdom(pose_i,d,phi)
% INTEGRATEODOM: given an initial configuration (within a global frame), the
% distance traveled and the angle turned, calculates the new configuration
% (in global coordinates) of a differential drive robot
% 
%   [POSE_F] = INTEGRATEODOM(POSE_I,D,PHI)
% 
%   INPUTS
%       pose_i      initial robot configuration in global frame, 1x3
%       d           distance traveled by robot, 1x1 [m]
%       phi         angle turned by robot, 1x1 [rad]
% 
%   OUTPUTS
%       pose_f      final configuration of robot in global frame, 1x3
%  
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #4
%   Pu, Kenneth (kp295) 

% Calculate x and y offsets, dx and dy
if (phi == 0)
    % If robot is traveling in a straight line (phi=0), dx and dy can 
    % be estimated using the distance traveled and the robot's current
    % orientation
    dx = d*cos(pose_i(3));
    dy = d*sin(pose_i(3));
else
    % If robot is traveling along an arc, find the length of the
    % corresponding chord, which can then be used to estimate dx and dy
    radius = d/phi;
    half_phi = phi/2;
    chord = 2*radius*sin(half_phi);

    dx = chord*cos(pose_i(3)+half_phi);
    dy = chord*sin(pose_i(3)+half_phi);
end

% Add dx, dy, and phi to initial configuration to get new configuration
pose_f = pose_i + [dx dy phi];