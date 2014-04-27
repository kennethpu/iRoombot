function[V w] = feedbackLin(theta, V_x, V_y, epsilon)
% FEEDBACKLIN: Transforms arbitrary V_x and V_y commands to corresponding
%              V and w commands. Assume the V_x and V_y commands are given
%              with respect to the inertial frame
% 
%   [V w] = FEEDBACKLIN(V_x, V_y) runs 
% 
%   INPUTS
%       theta   Initial orientation of robot
%       V_x     Target velocity in x direction (in inertial frame)
%       V_y     Target velocity in y direction (in inertial frame)
%       epsilon Distance from center point (for calculating angVel)
% 
%   OUTPUTS
%       V   Target forward velocity
%       w   Target angular rotation
%
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295) 

%===================================================================
% Convert V_x and V_y to V and w commands
%===================================================================
% V_x and V_y treated as inertial
V = V_x*cos(theta) + V_y*sin(theta);
w = (1/epsilon)*(-V_x*sin(theta) + V_y*cos(theta));

% V_x and V_y treated as body-fixed
%V = V_x;
%w = (1/epsilon)*V_y;