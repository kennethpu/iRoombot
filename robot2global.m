function[xyG] = robot2global(pose,xyR)
% ROBOT2GLOBAL: transform a 2D point in robot coordinates into global
% coordinates (assumes planar world).
% 
%   XYG = ROBOT2GLOBAL(POSE,XYR) returns the 2D point in global coordinates
%   corresponding to a 2D point in robot coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyR     2D point in robot coordinates (1-by-2)
% 
%   OUTPUTS
%       xyG     2D point in global coordinates (1-by-2)
% 
% 
%   Cornell University
%   MAE 4180/5180: Autonomous Mobile Robots
%   Homework #1
%   Pu, Kenneth (kp295)

% Extract x, y, and theta variables from pose
x = pose(1);
y = pose(2);
theta = pose(3);

% Calculate transform matrix to convert from local to global coordinates
R_ib = [cos(theta) -sin(theta); sin(theta) cos(theta)];
T_ib = [R_ib [x; y]; 0 0 1];

% Apply transformation matrix to local point to get global coordinates xyG
xyG = T_ib * [xyR 1].';

% Convert xyG into 1-by-2 matrix to fit output spec
xyG = xyG(1:2,1).';