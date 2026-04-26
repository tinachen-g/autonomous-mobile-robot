function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   LOPEZ, DELANEY 

% r = fwdVel/angVel;
% vL = angVel*(r-Wheel2center);
% vR = angVel*(r+Wheel2center);

% calculate left and right wheel velocity
vL = fwdVel - wheel2Center*angVel;
vR = fwdVel + wheel2Center*angVel;

% find maximum speed out of wheels
abs_wheels = abs([vL, vR]);
max_wheels = max(abs_wheels);

% scale both wheel speeds if greater than maxV
if max_wheels >= maxV
    scale = maxV/max_wheels;
    cmdV = fwdVel*scale;
    cmdW = angVel*scale;
else
    cmdV = fwdVel;
    cmdW = angVel;
end