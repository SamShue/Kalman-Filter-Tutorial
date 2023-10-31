clc;
clear all;
close all;

% time between measurements (seconds)
dt = 0.5;
% acceleration (meters / seconds^2)
a = 1;

% State vector - describes position, velocity
x = [0; 0];
% Covariance matrix
P = eye(2);
% State transition matrix
F = [1 dt; 0 1];
% Control vector - acceleration
u = [a];
% Control coefficient matrix - maps control vector into state vector space
B = [0.5*dt*dt; dt];
% Process noise matrix
Q = [0.25 0; 0 0.25];

% Measurement vector - velocity
z = [a*dt];
H = [0 1];
R = [1];

% Prediction step
x = F*x + B*u
P = F*P*F' + Q

% Update step
K = P*H'*inv(H*P*H' + R);
x = x + K*(z - H*x)
P = (eye(2) - K*H)*P*(eye(2) - K*H)' + K*R*K';