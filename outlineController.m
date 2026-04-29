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


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;
wheel2Center = 0.13;
maxV = 0.2;

phase = 'SPIN';
phaseStart = 0;


% dead-reckoning
x = 0;
y = 0;
theta = 0;

% particle filter
N_particles = 300;
N_orient = 50;

disp(waypoints);
particles = seedParticles(waypoints, N_particles);
weights = ones(N_particles, 1) / N_particles;

% localization bookkeeping
localized = false;
estimatedPose = [];
accumulatedBeacons = [];   % rows: [tagNum, bearing_cam]  (camera frame bearing)

SPIN_RATE = 0.4;
SPIN_DURATION = (2*pi / SPIN_RATE) * 2; % two spins

EXPLORE_DIST = 3.0; % meters per exploration step
EXPLORE_SPEED = 0.15; % m/s while exploring
MAX_INIT_TIME = 10;

% minority-particle fraction kept spread across all waypoints after resample
% so the filter can recover if it locked to the wrong starting position
FRAC_MINORITY = 0.10;

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

        % dead-reckoning update
        theta = mod(theta + deltaA + pi, 2*pi) - pi; 
        % fprintf('theta = %.4f\n', theta); 
        x = x + deltaD*cos(theta);
        y = y + deltaD*sin(theta);

        % store raw dead-reckoning path
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
                accumulatedBeacons = [accumulatedBeacons; tagId, bearing_cam]; %#ok
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