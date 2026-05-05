function particles_new = PF(particles, u, z, predict_func, likelihood_func)
% PF: One iteration of a generic Particle Filter
%
%   INPUTS
%       particles       N-by-4 matrix [x, y, theta, weight] for each particle
%       u               Control input [d; phi] (odometry increment)
%       z               Measurement vector [m x 1]
%       predict_func    @(particles, u) -> N-by-3 predicted [x,y,theta] for all particles
%       likelihood_func @(particle_pose, z) -> scalar likelihood weight
%
%   OUTPUTS
%       particles_new   N-by-4 resampled particle set [x, y, theta, weight]
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 5

    N = size(particles, 1);

    %% ============================================================
    %  STEP 1: PREDICT
    %  propagate each particle through motion model + add noise
    %  ============================================================
    predicted_poses = predict_func(particles(:,1:3), u);  % [N x 3]

    %% ============================================================
    %  STEP 2: UPDATE WEIGHTS
    %  compute likelihood of measurement for each particle
    %  ============================================================
    weights = zeros(N, 1);
    for i = 1:N
        weights(i) = likelihood_func(predicted_poses(i,:)', z);
    end

    %% ============================================================
    %  STEP 3: NORMALIZE WEIGHTS
    %  ============================================================
    w_sum = sum(weights);
    if w_sum < 1e-300
        % all weights collapsed so reset to uniform to avoid NaN
        weights = ones(N,1) / N;
    else
        weights = weights / w_sum;
    end

    %% ============================================================
    %  STEP 4: SYSTEMATIC RESAMPLING
    %  draw N new particles proportional to weights
    %  ============================================================
    cumW = cumsum(weights);
    r = rand() / N;
    step = 1 / N;

    new_idx = zeros(N, 1);
    j = 1;
    for i = 1:N
        target = r + (i-1) * step;
        while target > cumW(j) && j < N
            j = j + 1;
        end
        new_idx(i) = j;
    end

    particles_new = [predicted_poses(new_idx, :), weights(new_idx)];

    % re-normalize weights after resampling
    particles_new(:,4) = particles_new(:,4) / sum(particles_new(:,4));
end