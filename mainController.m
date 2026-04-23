function[dataStore] = mainController(Robot,maxTime)
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

data        = load('practicemap2025update.mat');
map         = data.map;
optWalls    = data.optWalls;
waypoints   = data.waypoints;
ECwaypoints = data.ECwaypoints;
beaconLoc   = data.beaconLoc;

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose',          [], ...
                   'odometry',           [], ...
                   'rsdepth',            [], ...
                   'bump',               [], ...
                   'beacon',             [], ...
                   'odomRelTraj',        [], ...   % raw dead-reckoning path [x y], grown every tick
                   'localisedAtOdom',    [], ...   % [x y th] odom snapshot when pose first locked
                   'visitedWaypoints',   [], ...   % [x y] confirmed waypoint visits
                   'visitedECWaypoints', [], ...   % [x y] confirmed EC waypoint visits
                   'optWallStatus',      zeros(size(optWalls,1),1), ... % 0=unresolved 1=present -1=absent
                   'ppSubstate',         [], ...
                   'ppTarget',           [], ...
                   'ppTag',              [], ...
                   'ppBearing',          [], ...
                   'exploreSubstate',    [], ...
                   'wallFollowStart',    []);


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
% disp(particles);
weights = ones(N_particles, 1) / N_particles;

% localization bookkeeping
localized = false;
estimatedPose = [];
accumulatedBeacons = [];   % rows: [tagNum, bearing_cam]  (camera frame bearing)

SPIN_RATE = 0.7;
SPIN_DURATION = (2*pi / SPIN_RATE) * 1.15;

EXPLORE_DIST = 3.0; % meters per exploration step
EXPLORE_SPEED = 0.15; % m/s while exploring
MAX_INIT_TIME = 10;

% minority-particle fraction kept spread across all waypoints after resample
% so the filter can recover if it locked to the wrong starting position
FRAC_MINORITY = 0.10;

% visualizePF(particles, weights, accumulatedBeacons, beaconLoc);


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

        % store raw dead-reckoning path; updateLivePlot anchors this to
        % estimatedPose so the trajectory shape is preserved across pose resets
        dataStore.odomRelTraj = [dataStore.odomRelTraj; x, y];
    catch
        disp('[PHASE I] Error retrieving or saving odometry data.');
    end
    % 
    % try
    %     latestDepth = dataStore.rsdepth(end, 2:end);   % strip timestamp
    %     depthAngles = linspace(-pi/2, pi/2, length(latestDepth));  % adjust to your RS FOV
    % catch
    %     latestDepth = [];
    %     depthAngles = [];
    % end
    % 
    % bumpTriggered = ~isempty(dataStore.bump) && any(dataStore.bump(end, 2:end));

    %% BEACON observations (CHECK DURING OPEN LAB)
    try
        [beacX, ~, beacZ, ~, Ntag] = ReadBeacon(Robot);
        if ~isempty(Ntag)
            for i = 1:length(Ntag)
                % Camera frame: Z is forward (robot +X), X is left (robot +Y)
                % Bearing in camera frame (angle from robot forward axis)
                bearing_cam = -atan2(beacX(i), beacZ(i));   % radians, + = left
                accumulatedBeacons = [accumulatedBeacons; Ntag(i), bearing_cam]; %#ok

            end

            % if ~isempty(accumulatedBeacons)
            %     fprintf('Detected beacons:\n');
            %     for i = 1:size(accumulatedBeacons, 1)
            %         fprintf('Tag %d -> Bearing: %.3f rad, ', ...
            %             accumulatedBeacons(i,1), accumulatedBeacons(i,2));
            %     end
            %     disp('\n =================== \n');
            % 
            % end

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
            % Update particle weights from accumulated beacon observations         
            uniqueTags = [];      
            if ~isempty(accumulatedBeacons)
                uniqueTags = unique(accumulatedBeacons(:, 1));
            end
            nUnique = numel(uniqueTags);
            disp('UNIQUETAGS');
            disp(uniqueTags);
           
         
            % --- whether or not beacons detected: update weights normally ---
            weights = updateWeightsBeacon(particles, weights, accumulatedBeacons, beaconLoc, waypoints);
            weights = weights / sum(weights);
            fprintf('[Phase1] Beacon weights updated. Tags seen: %d\n', nUnique);
            % visualizePF(particles, weights, uniqueTags, beaconLoc);

            % check 
            [converged, bestPose, confidence] = checkConvergence(particles, weights, waypoints);
            fprintf('\n[PHASE I] Convergence check — confidence=%.3f, tags=%d\n', ...
                    confidence, nUnique);
 
            % toc > MAX_INIT_TIME
            if converged 
                estimatedPose = bestPose;
                localized = true;
                phase = 'DONE';
                fprintf('[Phase1] LOCALIZED at x=%.2f y=%.2f theta=%.2f\n', ...
                        bestPose(1), bestPose(2), bestPose(3));

                % snapshot the dead-reckoning state at lock time so the plot
                % can rigidly transform the whole odom path into world frame
                if isempty(dataStore.localisedAtOdom)
                    dataStore.localisedAtOdom = [x, y, theta];
                end

                % resample with minority retention so the filter stays
                % recoverable if we locked to the wrong starting position
                nMinority   = round(N_particles * FRAC_MINORITY);
                nMajority   = N_particles - nMinority;
                weights = weights(:) / sum(weights);
                cdf = cumsum(weights);
                
                r = rand(nMajority, 1);
                [~, idx_majority] = histc(r, [0; cdf]);
                pMajority    = particles(idx_majority, :);
                pMinority    = seedParticles(waypoints, nMinority);
                particles    = [pMajority; pMinority];
                weights      = ones(N_particles, 1) / N_particles;

            elseif nUnique >= 1
                % -------- GO TO A BEACON AND DETERMINE LOCATION
                phase = 'PINPOINT';
                


            else
                % no beacons detected at all and no waypoint aligns with
                % this. need to pick a direction? 
                phase = 'EXPLORE';
                phaseStart = toc;
                fprintf('[Phase1] Not converged. NEED TO EXPLORE.\n');

                % ================= TODO =================:
                moveDir = chooseMoveDirection(particles, weights, beaconLoc, ...
                                          accumulatedBeacons, theta, dataStore);
                driveTime = EXPLORE_DIST / EXPLORE_SPEED;
            end

        case 'PINPOINT'
        if ~isfield(dataStore, 'ppSubstate') || isempty(dataStore.ppSubstate)
            dataStore.ppSubstate = 'RESPIN';
            dataStore.ppTarget = [];
            accumulatedBeacons = [];  % clear stale bearings
            fprintf('[PINPOINT] Starting fresh spin to find beacon\n');
        end
    
        switch dataStore.ppSubstate
    
            case 'RESPIN'
                % spin slowly; stop the moment any beacon is detected
                [beacX, ~, beacZ, ~, Ntag] = ReadBeacon(Robot);
                if ~isempty(Ntag)
                    % pick the one with smallest |bearing| (most forward)
                    bearings = -atan2(beacX, beacZ);
                    [~, bestIdx] = min(abs(bearings));
                    tagNum = Ntag(bestIdx);
                    row = find(beaconLoc(:,1) == tagNum, 1);
    
                    dataStore.ppTarget   = beaconLoc(row, 2:3);
                    dataStore.ppTag      = tagNum;
                    dataStore.ppBearing  = bearings(bestIdx);
                    dataStore.ppSubstate = 'ALIGN';
    
                    SetFwdVelAngVelCreate(Robot, 0, 0);
                    fprintf('[PINPOINT] Beacon %d detected, bearing=%.3f rad\n', ...
                            tagNum, bearings(bestIdx));
                else
                    % keep spinning
                    [cmdV, cmdW] = limitCmds(0, SPIN_RATE * 0.6, maxV, wheel2Center);
                    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
                end
    
            case 'ALIGN'
                % keep reading beacon bearing live and correct heading
                [beacX, ~, beacZ, ~, Ntag] = ReadBeacon(Robot);
                if ~isempty(Ntag)
                    idx = find(Ntag == dataStore.ppTag, 1);
                    if ~isempty(idx)
                        liveBearing = -atan2(beacX(idx), beacZ(idx));
                        if abs(liveBearing) < 0.08  % ~5 degrees, robot is facing beacon
                            SetFwdVelAngVelCreate(Robot, 0, 0);
                            dataStore.ppSubstate = 'DRIVE';
                            fprintf('[PINPOINT] Aligned. Driving toward beacon %d\n', ...
                                    dataStore.ppTag);
                        else
                            % correct toward beacon using live bearing
                            cmdW = 1.5 * liveBearing;
                            [~, cmdW] = limitCmds(0, cmdW, maxV, wheel2Center);
                            SetFwdVelAngVelCreate(Robot, 0, cmdW);
                        end
                    else
                        % lost the target tag — re-spin
                        dataStore.ppSubstate = 'RESPIN';
                    end
                else
                    dataStore.ppSubstate = 'RESPIN';
                end
    
            case 'DRIVE'
                bumped = ~isempty(dataStore.bump) && any(dataStore.bump(end, 2:end));
                if bumped
                    % ============ TODO ===============
                    % need to check whether actually at the tag?
                    % --- Hard pose reset ---
                    bx = dataStore.ppTarget(1);
                    by = dataStore.ppTarget(2);
                    headingToBeacon = atan2(by - y, bx - x);
                    x = bx - cos(headingToBeacon) * 0.05;
                    y = by - sin(headingToBeacon) * 0.05;
                    theta = headingToBeacon;
    
                    particles(:,1) = x + 0.04*randn(N_particles,1);
                    particles(:,2) = y + 0.04*randn(N_particles,1);
                    particles(:,3) = theta + 0.08*randn(N_particles,1);
                    weights = ones(N_particles,1) / N_particles;
    
                    estimatedPose = [x, y, theta];
                    localized = true;
                    dataStore.ppSubstate = [];
                    SetFwdVelAngVelCreate(Robot, 0, 0);
                    phase = 'PLANNING';
                    fprintf('[PINPOINT] Bumped! Pose fixed at (%.2f,%.2f,%.2f)\n', ...
                            x, y, theta);

                    % snapshot the dead-reckoning state at lock time so the plot
                    % can rigidly transform the whole odom path into world frame
                    if isempty(dataStore.localisedAtOdom)
                        dataStore.localisedAtOdom = [x, y, theta];
                    end

                    % resample with minority retention so the filter stays
                    % recoverable if the bump was against the wrong wall
                    nMinority    = round(N_particles * FRAC_MINORITY);
                    nMajority    = N_particles - nMinority;
                    idx_majority = randsample(N_particles, nMajority, true, weights);
                    pMajority    = particles(idx_majority, :);
                    pMinority    = seedParticles(waypoints, nMinority);
                    particles    = [pMajority; pMinority];
                    weights      = ones(N_particles, 1) / N_particles;

                else
                    % drive straight; use live beacon bearing to stay on course
                    [beacX, ~, beacZ, ~, Ntag] = ReadBeacon(Robot);
                    cmdV = EXPLORE_SPEED;
                    cmdW = 0;
                    if ~isempty(Ntag)
                        idx = find(Ntag == dataStore.ppTag, 1);
                        if ~isempty(idx)
                            liveBearing = -atan2(beacX(idx), beacZ(idx));
                            cmdW = 1.0 * liveBearing;  % light correction while driving
                        end
                    end
                    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);
                    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
                end
        end
        
        case 'EXPLORE'
            elapsed = toc - phaseStart;
            bumped = ~isempty(dataStore.bump) && any(dataStore.bump(end, 2:end));
        
            if ~isfield(dataStore, 'exploreSubstate') || isempty(dataStore.exploreSubstate)
                dataStore.exploreSubstate = 'DRIVE';
            end
        
            switch dataStore.exploreSubstate
                case 'DRIVE'
                    if bumped
                        % back up, then wall-follow
                        SetFwdVelAngVelCreate(Robot, -EXPLORE_SPEED, 0);
                        pause(0.5);
                        % turn 90 degrees left to begin following wall on right
                        SetFwdVelAngVelCreate(Robot, 0, SPIN_RATE);
                        pause((pi/2) / SPIN_RATE);
                        SetFwdVelAngVelCreate(Robot, 0, 0);
                        dataStore.exploreSubstate = 'WALL_FOLLOW';
                        dataStore.wallFollowStart = toc;
                    elseif elapsed < driveTime
                        headingError = mod(moveDir - theta + pi, 2*pi) - pi;
                        cmdW = 2.0 * headingError;
                        cmdV = EXPLORE_SPEED * max(0.1, 1 - abs(headingError)/pi);
                        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);
                        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
                    else
                        SetFwdVelAngVelCreate(Robot, 0, 0);
                        phase = 'SPIN';
                        phaseStart = toc;
                        dataStore.exploreSubstate = [];
                    end
        
                case 'WALL_FOLLOW'
                    wallElapsed = toc - dataStore.wallFollowStart;
                    % wall-follow for up to 8 seconds, then spin again
                    if wallElapsed > 8.0 || ~isempty(accumulatedBeacons)
                        SetFwdVelAngVelCreate(Robot, 0, 0);
                        phase = 'SPIN';
                        phaseStart = toc;
                        dataStore.exploreSubstate = [];
                    elseif bumped
                        SetFwdVelAngVelCreate(Robot, 0, SPIN_RATE);
                        pause(0.3);
                    else
                        % simple right-hand wall follow: drive forward, slight right bias
                        SetFwdVelAngVelCreate(Robot, EXPLORE_SPEED, -0.3);
                    end
            end

        case 'PLANNING'
            % ================= TODO =================:
            % now we know where we are in the world
            % implement algorithm to visit each waypoint



        case 'DONE'
            % fprintf('\nDONE');
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