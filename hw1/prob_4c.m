%% prob_4c.m
% 
% this is a MATLAB script to perform HW1.4 c
%
% - written by: Dimitri Lezcano

%% Set-up
global N_max epsilon 

N_max = 15000; % max # of iterations
epsilon = 0.005; % magnitude of dk to stop iterating
step_size = 0.001;
x_0 = [1.5; 3.0];

%% Perform gradient descent optimization
[x_grad, L_grad] = gradient_descent(@ cost_fn, @ dcost_fn, x_0, step_size);

%% perform the Newton method optimization

[x_new, L_new] = newton_method(@ cost_fn, @ dcost_fn, @hessian_cost_fn, x_0);

%% plot the results
% gradient descent
fgrad = figure(1);
subplot(2,1,1);
quiver(x_grad(1,:), x_grad(2,:), [diff(x_grad(1,:)), 0], [diff(x_grad(2,:)), 0],0, ...
    'Linewidth', 1.5); hold on
plot(x_grad(1,:), x_grad(2,:), 'o'); hold off;
xlabel('x1'); ylabel('x2');
title('trajectory');

subplot(2,1,2);
plot(L_grad,'Linewidth', 1.5)
xlabel('iteration'); ylabel('cost');
title('cost');

sgtitle(sprintf('4.a) gradient descent | %s = %.4f', '\alpha', step_size));

% Newton Method
fnew = figure(2);
subplot(2,1,1);
quiver(x_new(1,:), x_new(2,:), [diff(x_new(1,:)), 0], [diff(x_new(2,:)), 0],0, ...
    'Linewidth', 1.5); hold on;
plot(x_new(1,:), x_new(2,:), '.', 'markersize', 20); hold off;
% plot(x(1,:), x(2,:), '.'); hold off;
xlabel('x1'); ylabel('x2');
title('trajectory');

subplot(2,1,2);
plot(L_new, 'Linewidth', 1.5)
xlabel('iteration'); ylabel('cost');
title('cost');

sgtitle('4.a) Newton method');

%% saving
file_base = "hw1_prob_4c";

saveas(fgrad, file_base + "_grad.png");
disp('Saved figure ' + file_base + "_grad.png");

saveas(fnew, file_base + "_newton.png");
disp('Saved figure ' + file_base + "_newton.png");

%% Functions
% % plotting function for trajectory
% function plot_traj(x1, x2)
%     for i = 1:length(x1)
%         quiver(
%     
% end

% cost function for 4(a)
function L = cost_fn(x1, x2)
%     L = (1 - x1)..^2 + 200 .* (x2 - x1..^2)..^2;
    L = x1 .* exp(-x1.^2 - x2.^2/2) + x1.^2/10 + x2.^2/10;
    
end

% gradient for cost function of 4(a)
function dL = dcost_fn(x1, x2)
    dL = [ x1/5 + exp(- x1.^2 - x2.^2/2) - 2.*x1.^2.*exp(- x1.^2 - x2.^2/2);
           x2/5 - x1.*x2.*exp(- x1.^2 - x2.^2/2)];
        
end

% hessian for cost function of 4(a)
function d2L = hessian_cost_fn(x1, x2)
    d2L_x1_2 = 4.*x1.^3.*exp(- x1.^2 - x2.^2/2) - 6.*x1.*exp(- x1.^2 - x2.^2/2) + 1/5;
    d2L_x2_2 = x1.*x2.^2.*exp(- x1.^2 - x2.^2/2) - x1.*exp(- x1.^2 - x2.^2/2) + 1/5;
    d2L_x1_x2 = x2.*exp(- x1.^2 - x2.^2/2).*(2.*x1.^2 - 1);
    
    d2L = [d2L_x1_2, d2L_x1_x2;
           d2L_x1_x2, d2L_x2_2];
       
end

% the gradient descent returning a 2xN array of [x1; x2]
function [x_mat, L_mat] = gradient_descent(L, grad_L, x_0, step_size)
    global N_max epsilon

    % initialize the arrays to return
    x_mat = zeros(length(x_0), N_max);
    L_mat = zeros(N_max, 1); % array of losses
    
    % initial point
    x_mat(:,1) = x_0;
    L_mat(1) = L(x_0(1), x_0(2));
       
    for k = 2:N_max
        % determine the gradient
        grad_L_k = grad_L(x_mat(1,k-1), x_mat(2, k-1)); 
        
        % determine Dk matrix
        Dk = eye(length(grad_L_k));
        
        % determine dk, vector direction
        dk = Dk * grad_L_k;
        
        % take a step
        x_mat(:, k) = x_mat(:,k-1) - step_size .* dk;
        
        % calculate the cost
        L_mat(k) = L(x_mat(1,k), x_mat(2,k));
        
        % check if we have converged
        if norm(dk) < epsilon
            break;
        end
        
    end
    
    % return where we stopped
    x_mat = x_mat(:,1:k);
    L_mat = L_mat(1:k);
    
end

% newton-method implementation
function [x_mat, L_mat] = newton_method(L, grad_L, hess_L, x_0)
    global N_max epsilon

    % initialize the arrays to return
    x_mat = zeros(length(x_0), N_max);
    L_mat = zeros(N_max, 1); % array of losses
    
    % initial point
    x_mat(:,1) = x_0;
    L_mat(1) = L(x_0(1), x_0(2));
       
    for k = 2:N_max
        % determine the gradient
        grad_L_k = grad_L(x_mat(1,k-1), x_mat(2, k-1)); 
        
        % determine Dk matrix
        inv_Dk = hess_L(x_mat(1,k-1), x_mat(2,k-1));
        
        % determine dk, vector direction
        dk = inv_Dk \ grad_L_k;
        
        % take a step
        x_mat(:, k) = x_mat(:,k-1) - dk;
        
        % calculate the cost
        L_mat(k) = L(x_mat(1,k), x_mat(2,k));
        
        % check if we have converged
        if norm(dk) < epsilon
            break;
        end
        
    end
    
    % return where we stopped
    x_mat = x_mat(:,1:k);
    L_mat = L_mat(1:k);
    
end