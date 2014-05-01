function[xyR] = global2robot(pose,xyG)
% GLOBAL2ROBOT: transform a 2D point in global coordinates into robot
% coordinates (assumes planar world).
% 
%   XYR = GLOBAL2ROBOT(POSE,XYG) returns the 2D point in robot coordinates
%   corresponding to a 2D point in global coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyG     2D point in global coordinates (1-by-2)
% 
%   OUTPUTS
%       xyR     2D point in robot coordinates (1-by-2)
% 
%   Cornell University
%   MAE 4180/5180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

% Extract x, y, and theta variables from pose
x = pose(1);
y = pose(2);
theta = pose(3);

% Calculate transform matrix to convert from local to global coordinates
R_ib = [cos(theta) -sin(theta); sin(theta) cos(theta)];
T_ib = [R_ib [x; y]; 0 0 1];

% Apply inverse transfromation matrix to global point to get local coordinates xyR
xyR = T_ib \ [xyG 1].';

% Convert xyR into 1-by-2 matrix to fit output spec
xyR = xyR(1:2,1).';