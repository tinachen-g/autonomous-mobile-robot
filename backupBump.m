function [dataStore] = motionControl(Robot, maxTime)
% MOTIONCONTROL: drives robot while estimating pose with EKF.
%   Reacts to bump sensors. Control is deterministic and location-independent.
%
%   INPUTS
%       Robot       Port configurations and robot name (from simulator/CreatePiInit)
%       maxTime     max time to run program (seconds)
%
%   OUTPUTS
%       dataStore   struct containing logged data
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4

%% ============================================================
%  *** SWITCH BETWEEN ESTIMATION METHODS HERE ***
%
%     'GPS'   -> EKF with noisy GPS data  (Question 4)
%     'DEPTH' -> EKF with depth camera    (Question 5)
%
ESTIMATION_METHOD = 'GPS';
% ESTIMATION_METHOD = 'DEPTH';   % <-- uncomment for Question 5
%% ============================================================

% Set unspecified inputs
if nargin < 1
    disp('ERROR: Robot port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try
    % Real robot
    CreatePort = Robot.CreatePort;
catch
    % Simulator
    CreatePort = Robot;
end

% Declare dataStore as global so it's accessible even if program is stopped
global dataStore;

% Initialize dataStore struct
dataStore = struct('truthPose', [], ...
                   'odometry',  [], ...
                   'rsdepth',   [], ...
                   'bump',      [], ...
                   'beacon',    [], ...
                   'deadReck',  [], ...
                   'ekfMu',     [], ...
                   'ekfSigma',  [], ...
                   'GPS',       []);

%% ============================================================
%  MAP & SENSOR CONFIGURATION
%% ============================================================
load('cornerMap.mat');                  % loads variable 'map' [N x 4]
sensor_pos   = [0.13, 0];              % RealSense in robot frame [x, y] (m)
n_rs_rays    = 9;                      % number of depth rays
depth_angles = linspace(27*pi/180, -27*pi/180, n_rs_rays)';  % [n_rs_rays x 1]

%% ============================================================
%  NOISE COVARIANCE MATRICES
%  R:       process noise [3x3]          (motion model uncertainty)
%  Q_GPS:   GPS measurement noise [3x3]
%  Q_DEPTH: depth measurement noise [n_rs_rays x n_rs_rays]
%% ============================================================
R       = 0.01  * eye(3);
Q_GPS   = 0.001 * eye(3);
Q_DEPTH = 0.001 * eye(n_rs_rays);
gps_noise_std = sqrt(Q_GPS(1,1));

%% ============================================================
%  MOTION PARAMETERS
%% ============================================================
fwdVel       = 0.2;    % forward speed (m/s)
angVel_turn  = -0.5;   % turning speed (rad/s)
wheel2Center = 0.13;
maxV         = 0.4;
backupDist   = 0.20;
turnAngle    = pi/6;

%% ============================================================
%  INITIALIZE FILTER (after first sensor read)
%% ============================================================
noRobotCount = 0;
filterInit   = false;   % will init on first valid truthPose read
mu           = zeros(3,1);
sigma        = eye(3);

% Anonymous function pointers (created once, reused every step)
g_func      = @(x, u) integrateOdom(x, u(1), u(2));
G_jac_func  = @(x, u) GjacDiffDrive(x, u);
h_gps       = @(x) hGPS(x);
H_jac_gps   = @(x) eye(3);
h_depth     = @(x) depthPredict(x, map, sensor_pos(:), depth_angles);
H_jac_depth = @(x) HjacDepth(x, map, sensor_pos, n_rs_rays);

%% ============================================================
%  MAIN LOOP
%% ============================================================
SetFwdVelAngVelCreate(Robot, 0, 0);
tic

while toc < maxTime

    % --- Read all sensors ---
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);

    % --- Safety stop if localization lost ---
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0, 0);
        continue;
    end

    % --- Initialize filter on first valid reading ---
    if ~filterInit && ~isempty(dataStore.truthPose)
        true_init = dataStore.truthPose(1, 2:4)';   % [3x1]
        mu        = true_init;
        sigma     = [2, 0, 0; 0, 2, 0; 0, 0, 0.1];

        dataStore.deadReck        = [dataStore.truthPose(1,1), true_init'];
        dataStore.ekfMu           = [dataStore.truthPose(1,1), true_init'];
        dataStore.ekfSigma(:,:,1) = sigma;

        odom_prev  = dataStore.odometry(end, 2:3);  % [d_total, phi_total]
        filterInit = true;
        continue;
    end

    if ~filterInit
        continue;
    end

    % --- Incremental odometry: ut = [d; phi] ---
    odom_now  = dataStore.odometry(end, 2:3);
    ut        = [odom_now(1) - odom_prev(1); ...
                 odom_now(2) - odom_prev(2)];
    odom_prev = odom_now;
    t_now     = toc;

    % --- Dead reckoning ---
    pose_dr = integrateOdom(dataStore.deadReck(end, 2:4)', ut(1), ut(2));
    dataStore.deadReck(end+1, :) = [t_now, pose_dr'];

    % --- Noisy GPS measurement (zero-mean Gaussian noise ~ N(0, Q_GPS)) ---
    true_pose = dataStore.truthPose(end, 2:4)';
    z_gps     = true_pose + gps_noise_std * randn(3, 1);
    dataStore.GPS(end+1, :) = [t_now, z_gps'];

    % --- Depth measurement ---
    z_depth = dataStore.rsdepth(end, 2:end)';   % [n_rs_rays x 1]

    %% ----------------------------------------------------------
    %  EKF UPDATE — swap sensor by changing ESTIMATION_METHOD above
    %% ----------------------------------------------------------
    if strcmp(ESTIMATION_METHOD, 'GPS')
        [mu, sigma] = EKF(mu, sigma, ut, R, ...
                          z_gps, Q_GPS, ...
                          g_func, G_jac_func, h_gps, H_jac_gps);

    elseif strcmp(ESTIMATION_METHOD, 'DEPTH')
        [mu, sigma] = EKF(mu, sigma, ut, R, ...
                          z_depth, Q_DEPTH, ...
                          g_func, G_jac_func, h_depth, H_jac_depth);
    end

    dataStore.ekfMu(end+1, :)      = [t_now, mu'];
    dataStore.ekfSigma(:,:,end+1)  = sigma;

    %% ----------------------------------------------------------
    %  MOTION CONTROL: deterministic bump-reactive driving
    %% ----------------------------------------------------------
    bumped = false;
    if ~isempty(dataStore.bump)
        lastBump = dataStore.bump(end, 2:end);
        bumped   = any(lastBump);
    end

    if bumped
        % Back up
        t0 = tic;
        while toc(t0) < backupDist / fwdVel
            [cmdV, cmdW] = limitCmds(-fwdVel, 0, maxV, wheel2Center);
            SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        end
        SetFwdVelAngVelCreate(Robot, 0, 0);
        pause(0.1);

        % Turn right
        t0 = tic;
        while toc(t0) < abs(turnAngle / angVel_turn)
            [cmdV, cmdW] = limitCmds(0, angVel_turn, maxV, wheel2Center);
            SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        end
        SetFwdVelAngVelCreate(Robot, 0, 0);
        pause(0.1);
    end

    % Drive forward
    [cmdV, cmdW] = limitCmds(fwdVel, 0, maxV, wheel2Center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

end

SetFwdVelAngVelCreate(Robot, 0, 0);

end