clc;
clear all;
close all;

% time between measurements (seconds)
dt = 0.5;
% acceleration (meters / seconds^2)
a = 1;

% State vector - describes position, velocity in x and y ([x, dx, y, dy])
x = [0; 0; 0; 0];
% Covariance matrix
P = eye(4);
% State transition matrix
F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
% Control vector - acceleration in x and y ([ddx, ddy])
u = [a; 0];
% Control coefficient matrix - maps control vector into state vector space
B = [0.5*dt*dt 0; dt 0; 0 0.5*dt*dt; 0 dt];
% Process noise matrix
Q = [0.5 0.25 0 0; 0.25 0.5 0 0; 0 0 0.5 0.25; 0 0 0.25 0.5];

% Measurement vector - velocity in x and y ([dx, dy])
z = [a*dt; 0];
H = [0 1 0 0; 0 0 0 1];
R = [1 0; 0 1];

% Prediction step
x = F*x + B*u
P = F*P*F' + Q

% Update step
K = P*H'*inv(H*P*H' + R)
x = x + K*(z - H*x)
P = (eye(4) - K*H)*P*(eye(4) - K*H)' + K*R*K';