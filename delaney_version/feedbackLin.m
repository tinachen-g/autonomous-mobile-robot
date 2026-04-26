function [cmdV, cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)
% FEEDBACKLIN Transforms Vx and Vy commands into V and omega commands using
% feedback linearization techniques
% Inputs:
%  cmdVx: input velocity in x direction wrt inertial frame
%  cmdVy: input velocity in y direction wrt inertial frame
%  theta: orientation of the robot
%  epsilon: turn radius
% Outputs:
%  cmdV: fwd velocity
%  cmdW: angular velocity
%
% LOPEZ, DELANEY

% converts x and y velocity to forward and angular using matrix from class
cmdV = cmdVx*cos(theta)+cmdVy*sin(theta);
cmdW = (cmdVx*(-sin(theta))+cmdVy*cos(theta))/epsilon;