function [particles_new, best_particle, weights_new, predicted] = PF2(particles, u, z, predict_func, likelihood_func)
% PF2: one iteration of a particle filter with systematic resampling.
%
%   chosen over PF.m because systematic resampling has lower variance than
%   the random CDF inversion used in PF.m, and the predict/likelihood
%   function interface separates concerns cleanly.
%
%   inputs
%       particles        N-by-4 matrix [x, y, theta, weight]
%                        OR N-by-3 if weights are passed separately —
%                        in that case col-4 is added internally.
%       u                control input [deltaD; deltaA]
%       z                measurement: [] for uniform, or [M x 2] [tagID bearing]
%       predict_func     @(poses[Nx3], u) -> [Nx3]  motion model for all particles
%       likelihood_func  @(pose[1x3], z) -> scalar  measurement likelihood
%
%   outputs
%       particles_new    N-by-4 resampled set [x y theta weight]
%       best_particle    [1 x 3] highest-weight particle pose before resampling
%       weights_new      N-by-1 normalised weights after resampling
%       predicted        N-by-3 predicted poses before resampling
%
%   Cornell University — MAE 5180: Autonomous Mobile Robots

N = size(particles, 1);

% accept either [Nx3] or [Nx4] input
if size(particles,2) == 4
    weights_in = particles(:,4);
    poses_in   = particles(:,1:3);
else
    weights_in = ones(N,1) / N;
    poses_in   = particles(:,1:3);
end

% -------------------------------------------------------------------------
% step 1: predict — propagate through motion model
% -------------------------------------------------------------------------
predicted = predict_func(poses_in, u);   % [N x 3]

% -------------------------------------------------------------------------
% step 2: update weights via measurement likelihood
% -------------------------------------------------------------------------
weights = zeros(N,1);
for i = 1:N
    weights(i) = weights_in(i) * likelihood_func(predicted(i,:), z);
end

% -------------------------------------------------------------------------
% step 3: normalise
% -------------------------------------------------------------------------
w_sum = sum(weights);
if w_sum < 1e-300 || ~isfinite(w_sum)
    % weight collapse — reset to uniform so pf can recover
    weights = ones(N,1) / N;
else
    weights = weights / w_sum;
end

% -------------------------------------------------------------------------
% step 4: store best particle before resampling
% -------------------------------------------------------------------------
[~, best_idx]  = max(weights);
best_particle  = predicted(best_idx, :);   % [1 x 3]

% -------------------------------------------------------------------------
% step 5: systematic resampling
%   draws N samples proportional to weights with lower variance than
%   the multinomial (random CDF) method in PF.m.
% -------------------------------------------------------------------------
cumW = cumsum(weights);
cumW(end) = 1.0;          % numerical guard
r    = rand() / N;        % single random offset
step = 1.0 / N;

new_idx = zeros(N,1);
j = 1;
for i = 1:N
    target = r + (i-1)*step;
    while target > cumW(j) && j < N
        j = j + 1;
    end
    new_idx(i) = j;
end

resampled_poses   = predicted(new_idx, :);
resampled_weights = weights(new_idx);

% re-normalise after resampling
resampled_weights = resampled_weights / sum(resampled_weights);

particles_new = [resampled_poses, resampled_weights];
weights_new   = resampled_weights;
end