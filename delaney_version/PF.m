function [particles_next, best_particle, w_f, particle_pred] = PF(particles, u, z, pred_funct, meas_funct, Q)
% PF: one iteration of a particle filter
%
% INPUTS
%   particles    N x 3 matrix of particles [x y theta]
%   u            odometry input
%   z            measurement vector
%   pred_funct   function handle for motion model, e.g. @(x,u) integrateOdom(x,u)
%   meas_funct   function handle for measurement model, e.g. @(x) depthPredict(x,map,sensor_pos,angles)
%   Q            measurement noise covariance
%
% OUTPUT
%   particles_next   resampled particle set [n x 3]
%   best_particle    stores best particle before resampling
%   w_f              weights of particles
%   particle_pred    predicted particles before resampling

% initialize particle set
m = size(particles,1);
particle_pred = zeros(size(particles));
w_i = zeros(m,1);

% loop through particles to determine dynamics update and initial weights
for i = 1:m
    x_i = particles(i,:)';
    dyn_update = pred_funct(x_i, u);
    particle_pred(i,:) = dyn_update';

    meas_update = meas_funct(dyn_update);
    meas_diff = z - meas_update;
    w_i(i) = exp(-0.5*meas_diff'*(Q\meas_diff));
end

% normalize weights
weight_sum = sum(w_i);
if weight_sum <= 0 || any(~isfinite(w_i))
    w_f = ones(m,1)/m;
else
    w_f = w_i/weight_sum;
end
w_f = w_f(:);

% store best particle before resampling
[~, best_idx] = max(w_f);
best_particle = particle_pred(best_idx,:);

% cumulative distribution of weights before resampling
cdf = cumsum(w_f);
cdf(end) = 1;

resamp = rand(m,1);
w_samp = zeros(m,1);
% resample particles and update with weights
for j = 1:m
    idx = find(cdf >= resamp(j),1,'first');
    if isempty(idx)
        idx = m;
    end
    w_samp(j) = idx;
end

particles_next = particle_pred(w_samp,:);
