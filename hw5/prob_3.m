%% prob_3.m
%
% this script is for HW5 problem 3
%
% - written by: Dimitri Lezcano

clear; 

%% Set-up
N = 100;

% system
x0 = [10; -5];
S.Pf = eye(2);

S.R = 0.04;
S.dt = 0.1;

S.A = [1, S.dt; 0.2*S.dt, 1 - 0.5*S.dt];
S.B = [0; 1];
S.w = [0; 0.1];

% Value function params
S.P = cell(1,N);
S.b = cell(1,N);
S.c = cell(1,N);

S.P{N} = S.Pf;
S.b{N} = [0;0];
S.c{N} = 0;

% control law
S.K = cell(1,N);
S.k = cell(1,N);

% trajectory and control arrays
x = zeros(2,N);
x(:,1) = x0;

u = zeros(1,N);


%% Back integrate to get P_i, b_i, c_i, K_i, k_i
for i = N-1:-1:1

    % get P_i+1, b_i+1, c_i+1
    P_ip1 = S.P{i + 1};
    b_ip1 = S.b{i + 1};
    c_ip1 = S.c{i + 1};
    
    % determine K_i and k_i
    invR_BPB = inv(S.R + S.B' * P_ip1 * S.B); % helper inverse
    
    K_i = -invR_BPB * S.B' * P_ip1 * S.A;
    k_i = -invR_BPB * S.B' * (P_ip1 * S.w + b_ip1);
    
    % determine P_i, b_i, c_i
    P_i = (S.A + S.B * K_i)'*P_ip1*(S.A + S.B * K_i) + K_i'*S.R*K_i;
    b_i = (S.A + S.B * K_i)'*(P_ip1 * (S.w  + S.B*k_i) + b_ip1) + K_i'*S.R*k_i;
    c_i = 1/2 * (S.w + S.B * k_i)' * P_ip1 * (S.w + S.B * k_i) + b_ip1'*(S.w + S.B * k_i) + ...
        c_ip1 + 1/2 * k_i' * S.R * k_i;
    
    % assign the values
    S.K{i} = K_i;
    S.k{i} = k_i;
    
    S.P{i} = P_i;
    S.b{i} = b_i;
    S.c{i} = c_i;
    
end



%% Forward integrate using dynamics and new control law
for i = 1:N-1
    % get x_i to integrate to x_i+1
    x_i = x(:,i);  % i-th column
    
    % get the control law
    u_i = ctrl_law(S, x_i, i);
    
    % get x_i+1
    x_ip1 = dynamics(S, x_i, u_i, i);
    
    % add x_i+1 and u_i to the array
    x(:,i+1) = x_ip1;
    u(i) = u_i;
    
end

%% Plotting
f_traj = figure(1);
plot(x(1,:), x(2,:), 'DisplayName', 'trajectory'); hold on;
plot(x0(1), x0(2), 'r*', 'DisplayName', 'start'); hold off;
title(sprintf('Trajectory: R = %.3f', S.R));
xlabel('p'); ylabel('v');
legend();
grid on;

f_ctrl = figure(2);
plot(u);
title(sprintf('Control: R = %.3f', S.R));
xlabel('i'); ylabel('u');
grid on;

%% Saving
file_base = "prob_3_%s_" + sprintf("R-%.3f_x0_%d_%d", S.R, x0(1), x0(2));

saveas(f_traj, sprintf(file_base, 'traj') + ".png");
disp("Saved figure: " + sprintf(file_base, 'traj') + ".png");

saveas(f_traj, sprintf(file_base, 'ctrl') + ".png");
disp("Saved figure: " + sprintf(file_base, 'ctrl') + ".png");

%% Functions
% dynamics
function x_ip1 = dynamics(S, x_i, u_i, i)
    x_ip1 = S.A * x_i + S.B * u_i + S.w;
    
end

% control law
function u_i = ctrl_law(S, x, i)
    K_i = S.K{i};
    k_i = S.k{i};
    
    u_i = K_i * x + k_i;
    
end


