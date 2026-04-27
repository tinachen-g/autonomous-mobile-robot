function[dataStore] = testControllerOptionalWalls(Robot,maxTime)
% TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot turn in place and saves a datalog.
% 
%   dataStore = TURNINPLACE(Robot,maxTime) runs 
% 
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%
% 	Modified: Liran 2023


% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'wallBelief', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

mapfile = 'PracticeMap2026.mat';

S = load(mapfile);
numOptWalls = size(S.optWalls,1);

wallBelief.presentScore = zeros(numOptWalls,1);
wallBelief.absentScore = zeros(numOptWalls,1);
wallBelief.state = zeros(numOptWalls,1);  % 1 present, 0 unknown, -1 absent

sensorOrigin = [0 0.08];

numDepthRays = 9;
angles = linspace(deg2rad(27), -deg2rad(27), numDepthRays);

SetFwdVelAngVelCreate(Robot, 0,0);
tic
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)
    
    % Set angular velocity
    cmdV = 0;
    cmdW = 0.4;

    % call function to detect optional walls
    pose_est = dataStore.truthPose(end,2:4);
    depth_meas = dataStore.rsdepth(end,3:end);

    wallBelief = detectOptionalWalls(wallBelief, pose_est, depth_meas, ...
    mapfile, sensorOrigin, angles);

    dataStore.wallBelief = wallBelief;

    
    % Call limitCmds function HERE
    wheel2center = 0.13;
    maxV = 0.4;
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2center);


    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW );
    end
    
%     pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0 );
