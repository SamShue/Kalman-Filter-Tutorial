clc;
clear all;
close all;

acceleration_commands = [1, 0.5, 0.25, 0.15, 0.00, 0.00, 0.00, 0.0, 0;
                         0, 0.0, 0.00, 0.00, 0.00, 0.15, 0.25, 0.5, 1]

velocity_measurements = acceleration_commands .* 0.5

% State vector - describes position, velocity in x and y ([x, dx, y, dy])
x = [0; 0; 0; 0];
% Covariance matrix
P = eye(4);

for ii = 1:length(acceleration_commands)

    % Control vector - acceleration in x and y ([ddx, ddy])
    u = acceleration_commands(:,ii);
    
    % Measurement vector - velocity in x and y ([dx, dy])
    z = velocity_measurements(:,ii);

    [x, P] = KalmanFilter(x, P, u, z);

    % Render environment
    %======================================================================
    clf;
    hold on;
    xlim([-0.5 3.5]); ylim([-1.5 1.5]);
    xlabel('meters'); ylabel('meters');
    scatter(x(1), x(3), 'o', 'black', 'filled');
    pause(0.5);
    % End render environment
    %----------------------------------------------------------------------
end

function[x, P] = KalmanFilter(x, P, u, z)
    % time between measurements (seconds)
    dt = 0.5;
    % State transition matrix
    F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
    % Control coefficient matrix - maps control vector into state vector space
    B = [0.5*dt*dt 0; dt 0; 0 0.5*dt*dt; 0 dt];
    % Process noise matrix
    Q = [0.5 0.25 0 0; 0.25 0.5 0 0; 0 0 0.5 0.25; 0 0 0.25 0.5];
    % Observation matrix 
    H = [0 1 0 0; 0 0 0 1];
    % Measurement noise
    R = [1 0; 0 1];
    
    % Prediction step
    x = F*x + B*u
    P = F*P*F' + Q
    
    % Update step
    K = P*H'*inv(H*P*H' + R)
    x = x + K*(z - H*x)
    P = (eye(length(x)) - K*H)*P*(eye(length(x)) - K*H)' + K*R*K';
end