function f = shape_fit_ext
% static batch estimation of a shape defined as a quadratic
% function z = f(p,q) + v, and parametrized using a vector 
% 
% extended for the following:
%   - prior shape x0 ~ N(m0, P0)
%   - batch estimation
%       - iterative weighted least squares (4 iterations w/ 2/8 measurements each)
% - by: Dimitri Lezcano
% close all;

% workspace is the square [-s,s]x[-s,s]
s = 10;

% true shape parameter (i.e. a symmetric cup)
x_true =  [1; 1; 0; 0; 0; 0];


% plot true
% gt = ezsurf(@(p,q)shape(p, q, x_true),[-s,s]);
% alpha(gt, 0.3)

% prior determination
x0 = [1.2; 1.3; .2; .2; .2; .2]; % prior estimate
P0 = 16 * eye(6);

% measurement standard dev
std = 20;

% # of measurements
k = 8;

% generate random measurements
p = 4*s*(rand(k,1) - .5)
q = 4*s*(rand(k,1) - .5)
z = shape(p, q, x_true) + randn(k,1)*std

% estimate optimal parameters x
R = diag(repmat(std^2, k, 1));
H = shape_basis(p, q);
x_old = inv(H'*inv(R)*H)*H'*inv(R)*z


% iterative weighted least squares
meas_pairs = reshape(randperm(length(z)), 2, []); % pair off for weighted least squares
x_prior = x0;
P_prior = P0;
for i = 1:size(meas_pairs, 2)
    % grab measurments
    z_i = z(meas_pairs(:, i)); % grab a measurement pair
    H_i = H(meas_pairs(:, i), :); % grab measurement H
    R_i = diag(repmat(std^2, length(z_i), 1)); % measurement covariance
    
    p_i = p(meas_pairs(:,i));
    q_i = q(meas_pairs(:,i));
    
    z_prior = shape(p_i, q_i, x_prior);
    
    % Kalman update
    P = inv( inv(P_prior) + H_i'*inv(R_i)*H_i ); % posterior covariance
    K_i = P * H_i' * inv(R_i);                 % Kalman Gain
    x = x_prior + K_i * (z_i - z_prior);       % posterior shape
    
    % update the priors for the next iteration
    P_prior = P;
    x_prior = x;
end

fprintf('%10s | %10s | %10s\n', 'True', 'Original', 'Extended');
fprintf("% 10.4f | % 10.4f | % 10.4f\n", [x_true, x_old, x]')



% plot estimated
close all;
fig2 = figure(2);
hold on;
ge = fsurf(@(p, q)shape(p, q, x_old), [-s, s]);
alpha(ge, .8);
fsurf(@(p, q) shape(p, q, x_true), [-s, s]);
hold off;
title('Problem 4: Original Estimation');
view([-37.5, 30.0])
grid on;

fig3 = figure(3);
hold on;
ge = fsurf(@(p,q) shape(p,q,x),[-s,s]);
alpha(ge, .8)
fsurf(@(p, q) shape(p, q, x_true), [-s, s]);
hold off;
title("Problem 4: Extended Estimation | x_0 = [" + sprintf("%.1f, ", x0) + "]");
view([-37.5, 30.0])
grid on;


% saving
saveas(fig2, 'prob_4_original.png');
disp('Saved figure: prob_4_original.png');

saveas(fig3, 'prob_4_extended2.png');
disp('Saved figure: prob_4_extended2.png');




function f = shape_basis(p, q)
% quadratic function, although could be any shape
f = [p.^2, q.^2, p.*q, p, q, ones(size(p))];

function z = shape(p, q, x)
z = shape_basis(p, q)*x;

