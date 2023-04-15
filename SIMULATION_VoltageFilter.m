clc;
clear all;
close all;

sigma_control = 0.15;
sigma_measure = 0.1;

signal = sin(0:0.01:4*pi);

plot(signal);

signal_control = signal + normrnd(0, sigma_control, size(signal));
signal_measure = signal + normrnd(0, sigma_measure, size(signal));

signal_filtered = zeros(size(signal));

figure; plot(signal_control); 
figure; plot(signal_measure);

x = 0;
P = 1;

for ii = 1:length(signal_control)
    
    u = signal_control(ii);
    z = signal_measure(ii);

    [x, P] = KalmanFilter(x, P, u, z, sigma_control, sigma_measure);

    signal_filtered(ii) = x;

end

figure; plot(signal_filtered);

function[x, P] = KalmanFilter(x, P, u, z, sigma_u, sigma_z)
    % State transition matrix
    F = [0];
    % Control coefficient matrix - maps control vector into state vector space
    B = [1];
    % Process noise matrix
    Q = [sigma_u];
    % Observation matrix 
    H = [1];
    % Measurement noise
    R = [sigma_z];
    
    % Prediction step
    x = F*x + B*u
    P = F*P*F' + Q
    
    % Update step
    K = P*H'*inv(H*P*H' + R)
    x = x + K*(z - H*x)
    P = (eye(length(x)) - K*H)*P*(eye(length(x)) - K*H)' + K*R*K';
end