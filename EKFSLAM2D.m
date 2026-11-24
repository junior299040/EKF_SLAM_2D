% EKF-SLAM in 2D
% Initialization

clc; close all; clear;

% Model evolution noise
q = [0.005; 0.003];
Q = diag(q.^2);
% Measurement noise
m = [.25; 1*pi/180];
M = diag(m.^2);

% Loading landmarks
load('W.mat')

% System parameters
% R → robot pose (x, y, theta)
% u → control (delta_x, delta_y, delta_theta)
% y → map (W → landmarks positions from enclosed file)

R = [0; -2.5; 0];
u = [0.1; 0.05];
y = zeros(2, size(W, 2));

% EKF-state and covariance matrix
% state (robot pose and landmark positions)
x = zeros(numel(R)+numel(W), 1);
% covariance matrix
P = zeros(numel(x), numel(x));

mapspace = 1:numel(x);
l = zeros(2, size(W, 2));
r = find(mapspace, numel(R));
mapspace(r) = 0;
x(r) = R;
P(r, r) = 0;

%%%
% Graphics initialization
mapFig = figure(1);
cla;
axis([-6 6 -6 6])
axis square

% Draw landmarks
WG = line('parent', gca, ...
            'xdata', W(1,:), ...
            'ydata', W(2,:), ...
            'color', 'r', ...
            'marker', 'o', ...
            'linestyle', 'none');

% Draw initial robot position
RG = line('parent', gca, ...
            'xdata', R(1), ...
            'ydata', R(2), ...
            'color', 'r', ...
            'marker', '.');

% Draw initial robot position estimation
rG = line('parent', gca, ...
            'xdata', x(r(1)), ...
            'ydata', x(r(2)), ...
            'marker', '+', ...
            'color', 'b', ...
            'linestyle', 'none');

% Initialize objects for future landmarks drawing

lG = line('parent', gca, ...
            'xdata', [], ...
            'ydata', [], ...
            'marker', '+', ...
            'color', 'b', ...
            'linestyle', 'none');

eG = zeros(1, size(W,2));

for i = 1:numel(eG)
    eG(i) = line('parent', gca, 'xdata', [], 'ydata', [], 'color', 'g');
end

reG = line('parent', gca, 'xdata', [], 'ydata', [], 'color', 'm');

%%%
% EKF-SLAM simulation

for t = 1:500
    
    % TODO Simulate robot movement and landmarks observation
    noise = q .* randn(2,1); % simulate the noise
    R = move(R, u, noise); % new states of the robot

    % Observe all discovered landmarks
    for i = 1:min(t, size(W,2))

        % Compute polar measurement
        y(:,i) = back_project_from_map(R, W(:,i));

        % Add measurement noise
        y(:,i) = y(:,i) + m .* randn(2,1);

        % Normalize bearing angle
        y(2,i) = mod(y(2,i) + pi, 2*pi) - pi;

    end
    
    %%%

    % TODO Extended Kalman Filter
    % Prediction and correction with known landmarks

    % Prediction step

    % Robot state prediction
    [x(r), J_r, J_noise] = move(x(r), u, zeros(2,1));

    % State Jacobian
    F_x = eye(numel(x));
    F_x(r,r) = J_r;

    % Noise Jacobian
    F_n = zeros(numel(x), 2);
    F_n(r,:) = J_noise;

    % Covariance prediction
    P = F_x * P * F_x' + F_n * Q * F_n';

    % Correction step

    % Correct using previously mapped landmarks
    known_landmarks = find(l(1,:) ~= 0);
    for i = 1:length(known_landmarks)

        % Predicted observation
        [z_pred, H_R, H_L] = back_project_from_map(x(r), x(l(:,i)));

        % Observation Jacobian
        H_x = zeros(2, numel(x));
        H_x(:,r) = H_R;
        H_x(:,l(:,i)) = H_L;

        % Innovation
        innovation = y(:,i) - z_pred;

        % Normalize angular innovation
        innovation(2) = mod(innovation(2) + pi, 2*pi) - pi;

        % Innovation covariance
        Z = H_x * P * H_x' + M;

        % Kalman gain
        K = (P * H_x') / Z;

        % State correction
        x = x + K * innovation;

        % Normalize robot orientation
        x(r(3)) = mod(x(r(3)) + pi, 2*pi) - pi;

        % Covariance correction
        P = P - K * Z * K';

        % Ensure covariance symmetry
        P = (P + P') / 2;
    end
    
    %%%
    
    % TODO Check if there is new landmarks and addition to the map
    if t <= size(W,2)

        % Find available positions in the state vector
        idx = find(mapspace, 2);

        % Initialize landmark position
        [p, G_R, G_y] = measurement_to_map(x(r), y(:,t));

        % Cross covariance between new landmark and existing state
        P_Lx = G_R * P(r,:);

        % Covariance of the new landmark
        P_LL = G_R * P(r,r) * G_R' + G_y * M * G_y';

        % Add landmark to state vector
        x(idx) = p;

        % Update covariance matrix
        P(idx,:) = P_Lx;
        P(:,idx) = P_Lx';
        P(idx,idx) = P_LL;

        % Update landmark indices
        l(:,t) = idx;

        % Mark positions as occupied
        mapspace(idx) = 0;

    end
    
    %%%
    
    % Graphics drawing
    set(RG, 'xdata', R(1), 'ydata', R(2));
    set(rG, 'xdata', x(r(1)), 'ydata', x(r(2)));
    
    lids = find(l(1, :));
    
    lx = x(l(1, lids));
    ly = x(l(2, lids));
    
    set(lG, 'xdata', lx, 'ydata', ly);
    
    for lid = lids
        le = x(l(:, lid));
        LE = P(l(:, lid),l(:, lid));
        [X, Y] = cov_to_ellipse(le, LE, 3, 16);
        set(eG(lid), 'xdata', X, 'ydata', Y);
    end
    
    if t > 1
        re = x(r(1:2));
        RE = P(r(1:2), r(1:2));
        [X,Y] = cov_to_ellipse(re, RE, 3, 16);
        set(reG, 'xdata', X, 'ydata', Y);
    end
    
    drawnow;

end
