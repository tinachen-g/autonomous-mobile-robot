function[dataStore] = outlineController(Robot,maxTime)
% outlineController: simple skeleton program to use with iRobot Create (or simulator).
%
%   dataStore = OUTLINECONTROLLER(Robot,maxTime) runs 
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

MAPFILE = 'practicemap2025update.mat';
data = load(MAPFILE);
map = data.map;
optWalls = data.optWalls;
waypoints = data.waypoints;
ECwaypoints = data.ECwaypoints;
beaconLoc = data.beaconLoc;

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose',          [], ...
                   'odometry',           [], ...
                   'rsdepth',            [], ...
                   'bump',               [], ...
                   'beacon',             []);



SetFwdVelAngVelCreate(Robot, 0,0);
tic
while toc < maxTime

    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);

    %%  ODOMETRY readings
    try
        deltaD = DistanceSensorRoomba(CreatePort);
        deltaA = AngleSensorRoomba(CreatePort);

        % fprintf('deltaA = %.4f\n', deltaA); 
        dataStore.odometry = [dataStore.odometry; toc deltaD deltaA];

 
    catch
        disp('[PHASE I] Error retrieving or saving odometry data.');
    end

    %% BEACON observations (CHECK DURING OPEN LAB)
    try
        tags = RealSenseTag(Robot);
        if ~isempty(tags)
            for i = 1:size(tags, 1)
                tagId = tags(i, 2);
                z = tags(i, 3);
                x_cam = tags(i, 4);
                bearing_cam = -atan2(x_cam, z);  % same convention as before

            end
        end
    catch
        disp('[PHASE I] no beacon available.');
    end
    
    %% CONTROL FUNCTION (send robot commands)

    switch phase
        %TURN in place: robot 360- yes or no (want: 2 beacons)
        case 'SPIN'
            elapsed = toc - phaseStart;
            if elapsed < SPIN_DURATION
                [cmdV, cmdW] = limitCmds(0, SPIN_RATE, maxV, wheel2Center);
                SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

            else
                SetFwdVelAngVelCreate(Robot, 0, 0);
                phase = 'ANALYZE';
                phaseStart = toc;
                fprintf('[PHASE I] Spinning DONE.\n');
            end

        case 'ANALYZE'
        

        case 'PINPOINT'

        case 'EXPLORE'
        

        case 'PLANNING'
            % ================= TODO =================:
            % now we know where we are in the world
            % implement algorithm to visit each waypoint



        case 'DONE'
            SetFwdVelAngVelCreate(Robot, 0, 0);

    end

    % live plot — update every iteration, drawnow limitrate keeps it cheap
    updateLivePlot(particles, weights, estimatedPose, dataStore, ...
                   map, optWalls, waypoints, ECwaypoints, beaconLoc, phase);

    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 5
        SetFwdVelAngVelCreate(Robot, 0,0);
        break;
    end
    
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0 );