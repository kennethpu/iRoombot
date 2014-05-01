function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,robotRad)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,ROBOTRAD) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       robotRad    robot radius (in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
%  
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295) 

% Calculate Right and Left wheel velocities
r_WVel = fwdVel + angVel*robotRad;
l_WVel = fwdVel - angVel*robotRad;

% Calculate a scaling factor as the largest ratio between wheel velocity
% and max velocity
scale = max(abs(r_WVel)/maxV, abs(l_WVel)/maxV);

% If the scaling factor exceeds 1, divide both wheel speeds by the scaling
% factor, then calculate the corresponding cmdV and cmdW. Otherwise, cmdV
% and cmdW are simply fwdVel and angVel respectively
if (scale > 1)
    r_WVel = r_WVel/scale;
    l_WVel = l_WVel/scale;
    cmdV = (r_WVel+l_WVel)/2;
    cmdW = (r_WVel-l_WVel)/(2*robotRad);
else
    cmdV = fwdVel;
    cmdW = angVel;
end
