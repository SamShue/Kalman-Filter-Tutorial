clc;
clear all;
close all;

sigma_control = 0.15;
sigma_measure = 0.1;

signal = sin(0:0.01:4*pi);

subplot(2,2,1); plot(signal); title('Original Signal')

signal_control = signal + normrnd(0, sigma_control, size(signal));
signal_measure = signal + normrnd(0, sigma_measure, size(signal));

signal_filtered = zeros(size(signal)); 

subplot(2,2,2); plot(signal_control); title('Signal as sensed by sensor 1 (used in control)')
subplot(2,2,3); plot(signal_measure); title('Signal as sensed by sensor 2 (used in measurement)')

x = 0;
P = 1;

for ii = 1:length(signal_control)
    
    u = signal_control(ii);
    z = signal_measure(ii);

    [x, P] = KalmanFilter(x, P, u, z, sigma_control, sigma_measure);

    signal_filtered(ii) = x;

end

subplot(2,2,4); plot(signal_filtered); title('Filtered Signal')

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