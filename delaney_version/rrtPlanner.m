function[dataStore] = rrtPlanner(Robot, maxTime)
% RRTPLANNER: builds an RRT path once, then follows it with visitWaypoints
%
% INPUTS:
%   Robot       robot object / simulator object
%   maxTime     max runtime in seconds
%
% OUTPUT:
%   dataStore   logged sensor data

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

global dataStore;

% initialize datalog struct
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [],...
                   'rrtPath', [], ...
                   'rrtNodes', [], ...
                   'rrtParent', []);

noRobotCount = 0;

% stop robot initially
SetFwdVelAngVelCreate(Robot, 0, 0);

% ---------- SETTINGS ----------
goal = [2 2.5];
robot_radius = 0.2;

% you may need to adjust these if your map bounds are different
mapBoundary = [-5 -5 5 5];
% data = load('cornerMap.mat');
% map = data.map;
map = 'cornermap_RRT.txt';

closeEnough = 0.1;
epsilon = 0.1;

path = [];
gotopt = 1;
pathBuilt = false;
triedPlanning = false;

tic
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
    
    % wait until truthPose exists
    if isempty(dataStore.truthPose)
        SetFwdVelAngVelCreate(Robot, 0, 0);
        continue;
    end
    
    % build RRT once using current robot position as start
    if ~triedPlanning
        start = [dataStore.truthPose(end,2), dataStore.truthPose(end,3)];
    
        [path, V, parent] = buildRRT('cornermap_RRT.txt', mapBoundary, start, goal, robot_radius);
    
        triedPlanning = true;
    
        if isempty(path)
            disp('Planner returned empty path');
            cmdV = 0;
            cmdW = 0;
        else
            disp('Planner returned a valid path');
            pathBuilt = true;
            gotopt = 2;   % skip the start waypoint
            cmdV = 0;
            cmdW = 0;
    
            dataStore.rrtPath = path;
            dataStore.rrtNodes = V;
            dataStore.rrtParent = parent;
        end
    
    elseif pathBuilt
        [cmdV, cmdW, gotopt] = visitWaypoints(path, gotopt, closeEnough, epsilon);
    
        if gotopt > size(path,1)
            cmdV = 0;
            cmdW = 0;
        end
    
    else
    % already tried planning and failed
    cmdV = 0;
    cmdW = 0;
    end

    % limit commands
    wheel2center = 0.13;
    maxV = 0.4;
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2center);
    
    % if localization is lost, stop robot
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0, 0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
end

% stop before exiting
SetFwdVelAngVelCreate(Robot, 0, 0);

end