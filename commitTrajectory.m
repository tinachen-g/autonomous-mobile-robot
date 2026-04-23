function [dataStore, odomOriginInWorld, localizationCommitted] = ...
         commitTrajectory(dataStore, worldX, worldY, worldTheta, ...
                          odomX, odomY, odomTheta)
% COMMITTRAJECTORY  Retroactively transform the odom-frame trajectory into
%   world frame on first localization commit.
%
%   INPUTS
%     dataStore    global dataStore struct (with .trajectory field)
%     worldX/Y/Theta   confirmed world-frame pose at the commit moment
%     odomX/Y/Theta    corresponding odom-frame position at same moment
%
%   OUTPUTS
%     dataStore              updated — .trajectory now in world frame
%     odomOriginInWorld      [tx, ty, dTheta]  stored for diagnostics
%     localizationCommitted  true

% Guard: only transform once.
% Subsequent pose corrections update x,y,theta directly in the main loop.
if isfield(dataStore,'localizationCommitted') && dataStore.localizationCommitted
    localizationCommitted = true;
    odomOriginInWorld = dataStore.odomOriginInWorld;
    return;
end

% ── Rigid transform: odom frame → world frame ────────────────────────────
%
%   We know:    R * odomCurrent + t = worldCurrent
%   Solve for:  t = worldCurrent - R * odomCurrent
%
dTheta = worldTheta - odomTheta;
R = [cos(dTheta), -sin(dTheta);
     sin(dTheta),  cos(dTheta)];

t = [worldX; worldY] - R * [odomX; odomY];

odomOriginInWorld = [t(1), t(2), dTheta];

% ── Apply to every stored trajectory point ───────────────────────────────
traj = dataStore.trajectory;   % T×2, recorded in odom frame
if ~isempty(traj)
    % (R * traj')' gives T×2 rotated, then broadcast-add translation
    dataStore.trajectory = (R * traj')' + repmat(t', size(traj,1), 1);
end

% ── Persist so we never re-transform ─────────────────────────────────────
dataStore.localizationCommitted = true;
dataStore.odomOriginInWorld     = odomOriginInWorld;
localizationCommitted           = true;

fprintf('[TRAJECTORY] Commit applied to %d points.\n', size(traj,1));
fprintf('             Odom origin in world: (%.3f, %.3f, %.1f deg)\n', ...
        t(1), t(2), rad2deg(dTheta));
end