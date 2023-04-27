clc;
clear all;
close all;

% control signals
acceleration_commands = [1, 0.5, 0.25, 0.15, 0.00, 0.00, 0.00, 0.0, 0;
                         0, 0.0, 0.00, 0.00, 0.00, 0.15, 0.25, 0.5, 1];
% measurement signals
dt = 0.5;
velocity_measurements = zeros(size(acceleration_commands));
velocity_measurements(:,1) = acceleration_commands(:,1) .* dt;
for ii = 2:length(velocity_measurements)
    velocity_measurements(:,ii) = velocity_measurements(:, ii - 1) +  acceleration_commands(:, ii) .* dt;
end

% Gaussian noise std deviation
u_sigma = 0.05;
z_sigma = 0.01;

% Add 0-mean gaussian noise to signals
acceleration_commands = acceleration_commands + normrnd(0, u_sigma, size(acceleration_commands));
velocity_measurements = velocity_measurements + normrnd(0, z_sigma, size(velocity_measurements));

% State vector - describes position, velocity in x and y ([x, dx, y, dy])
x = [0; 0; 0; 0];
% Covariance matrix
P = eye(length(4)) * 5;

for ii = 1:length(acceleration_commands)

    % Control vector - acceleration in x and y ([ddx, ddy])
    u = acceleration_commands(:,ii);
    
    % Measurement vector - velocity in x and y ([dx, dy])
    z = velocity_measurements(:,ii);

    [x, P] = KalmanFilter(x, P, u, z, u_sigma, z_sigma);

    % Render environment
    %======================================================================
    clf;
    hold on;
    xlim([0.0 4]); ylim([-0.25 1]);
    xlabel('meters'); ylabel('meters');
    scatter(x(1), x(3), 'o', 'black', 'filled');
    ellipse(sqrt(P(1,1))/32, sqrt(P(3,3))/32, 0, x(1), x(3), 'r');
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
    Q = F*u_sigma*F';
    % Observation matrix 
    H = [0 1 0 0; 0 0 0 1];
    % Measurement noise
    R = H*z_sigma*H';
    
    % Prediction step
    x = F*x + B*u;
    P = F*P*F' + Q;
    
    % Update step
    y = z - H*x;
    S = H*P*H' + R;
    K = P*H'*inv(S)
    x = x + K*y
    P = (eye(length(x)) - K*H)*P
end