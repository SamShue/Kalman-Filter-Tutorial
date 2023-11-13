clc;
clear all;
close all;

% vector for tracking estimation history
x_hist = [];

% control signals
% commanded acceleration (meters / seconds^2)
acceleration_commands = [1, 0.5, 0.25, 0.15, 0.00, 0.00, 0.00, 0.0, 0;
                         0, 0.0, 0.00, 0.00, 0.00, 0.15, 0.25, 0.5, 1];
% measurement signals
% time between measurements (seconds)
dt = 0.5;
% measured velocity (meters / second)
velocity_measurements = zeros(size(acceleration_commands));
velocity_measurements(:,1) = acceleration_commands(:,1) .* dt;
for ii = 2:length(velocity_measurements)
    velocity_measurements(:,ii) = velocity_measurements(:, ii - 1) +  acceleration_commands(:, ii) .* dt;
end

% measured position (meters)
position_measurements = zeros(size(velocity_measurements));
position_measurements(:,1) = velocity_measurements(:,1) .* dt;
for ii = 2:length(position_measurements)
    position_measurements(:,ii) = position_measurements(:, ii - 1) +  velocity_measurements(:, ii) .* dt;
end


% Gaussian noise std deviation
u_sigma = 0.2;
z_sigma = 0.1;

% Add 0-mean gaussian noise to signals
acceleration_commands_noisy = acceleration_commands + normrnd(0, u_sigma, size(acceleration_commands));
position_measurements_noisy = position_measurements + normrnd(0, z_sigma, size(position_measurements));

% State vector - describes position, velocity in x and y ([x, dx, y, dy])
x = [0; 0; 0; 0];
% Covariance matrix
P = eye(4) * 5;

for ii = 1:length(acceleration_commands_noisy)
    % Control vector - acceleration in x and y ([ddx, ddy])
    u = acceleration_commands_noisy(:,ii);
    
    % Measurement vector - velocity in x and y ([dx, dy])
    z = position_measurements_noisy(:,ii);

    [x, P] = KalmanFilter(x, P, u, z, u_sigma, z_sigma);

    % Render environment
    %======================================================================
    x_hist = [x_hist, x];
    clf;
    hold on;
    xlim([0.0 4]); ylim([-0.25 1]);
    xlabel('meters'); ylabel('meters');
    plot(x_hist(1, :), x_hist(3, :),'--o');
    plot(position_measurements(1,1:ii), position_measurements(2,1:ii), '-o');
    legend('estimated position', 'true position')
    pause(1);
    % End render environment
    %----------------------------------------------------------------------
end

function[x, P] = KalmanFilter(x, P, u, z, u_sigma, z_sigma)
    % time between measurements (seconds)
    dt = 0.5;
    % State transition matrix
    F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
    % Control coefficient matrix - maps control vector into state vector space
    B = [0.5*dt*dt 0; dt 0; 0 0.5*dt*dt; 0 dt];
    % Process noise matrix
    Q = B*u_sigma*B';
    % Observation matrix 
    H = [1 0 0 0; 0 0 1 0];
    % Measurement noise
    R = H*z_sigma*H';
    
    % Prediction step
    x = F*x + B*u;
    P = F*P*F' + Q;
    
    % Update step
    y = z - H*x;
    S = H*P*H' + R;
    K = P*H'*inv(S);
    x = x + K*y;
    P = (eye(length(x)) - K*H)*P
end