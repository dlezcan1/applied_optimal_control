%% int_kf_gyro.m
%
% this is a script to handle part 2.b of homework 8
%
% - written by: Dimitri Lezcano

%% script
function f = int_kf_gyro
% Kalman filtering of the gyrometer with position measurements

% timing
dt = 1;   % time-step
N = 100;   % total time-steps
T = N*dt;  % final time

% noise terms
S.sig_v2 = 3E-6;      % external theta disturbance
S.sig_u2 = 3E-9;      % external beta  disturbance
S.sig_n2 = 1.5E-5;    % measurement noise variance

S.sig_v = sqrt(S.sig_v2);
S.sig_u = sqrt(S.sig_u2);
S.sig_n = sqrt(S.sig_n2);

% F matrix
S.F = [1 -dt;
       0 1];

% G (gamma) matrix
S.G = [dt; 
       0];

% Q matrix
S.Q = [dt*S.sig_v2 + dt^3*S.sig_u2/3, -dt^2*S.sig_u2/2;
       -dt^2*S.sig_u2/2,                dt*S.sig_u2];

% R matrix (measurement covariance)
S.R = S.sig_n2;

% H matrix (from measurement)
S.H = [1, 0];

% control terms
w = 0.02;  % known actual control

% initial estimate of mean and covariance
x = [0; 1.7E-7];
P = diag([1E-4 1E-12]);

xts = zeros(2, N+1); % true states (with bias)
xs = zeros(2, N+1);  % estimated states
Ps = zeros(2, 2, N+1); % estimated covariances
b = zeros(1, N+1); % true bias term
b(1) = x(2); % initial condition

zs = zeros(1, N);  % estimated state

pms = zeros(1, N); % measured position

xts(:,1) = x;
xs(:,1) = x;
Ps(:,:,1) = P;

for k=1:N
  db = S.sig_u*randn; % next drift term
  
  u = w + xts(2,k); % pick some known control with drift

  xts(:,k+1) = S.F*xts(:,k) + S.G*(u + S.sig_v*randn) + [0; db];  % true state

  [x,P] = kf_predict(x,P,u,S);  % prediction
  
  z = xts(1,k+1) + S.sig_n*randn;   % generate random measurement 
  
  [x,P] = kf_correct(x,P,z,S);  % correction
  
  % record result
  xs(:,k+1) = x;
  Ps(:,:,k+1) = P;
  zs(:,k) = z;
end

fig = figure(1);
plot(xts(1,:), '--', 'LineWidth',2); hold on;
hold on
plot(xs(1,:), 'g', 'LineWidth',2); hold on;
plot(2:N+1,zs(1,:), 'r', 'LineWidth',2); hold on;

legend('true', 'estimated','measured')
title('Problem 2b');

% 95% confidence intervals of the estimated position
plot(xs(1,:) + 1.96*reshape(sqrt(Ps(1,1,:)),N+1,1)', '-g'); hold on;
plot(xs(1,:) - 1.96*reshape(sqrt(Ps(1,1,:)),N+1,1)', '-g'); hold off;

% saving
saveas(fig, 'prob_2b.png');
disp('Saved figure: prob_2.png');



%% Functions
% Kalman filtering functions
function [x,P] = kf_predict(x, P, u, S)

x = S.F*x + S.G*u;
P = S.F*P*S.F' + S.Q;

function [x,P] = kf_correct(x, P, z, S)

K = P*S.H'*inv(S.H*P*S.H' + S.R);
P = (eye(length(x)) - K*S.H)*P;
x = x + K*(z - S.H*x);
