%% ekf_car
%
% script to perform Kalman filtering on the car model
%
% - written by: Dimitri Lezcano

clear;

%% Set-Up
% saving
save_dir = "results/";
save_base = save_dir + "oscillating-ctrl-turning";

% system constants
S.l = 1;
S.Re = 6.38e6; % radius of earth in meters

% System functions
S.f = @ car_f; % dynamics 
S.h = @ car_h; % measurement

% time
S.dt = 0.05;
t = 0:S.dt:50;
N = length(t);

% known control set-up
S.w = 1;
S.u0 = [0.5;0.1];

% initial condition
x_true0 = [1; 0; pi/4; 0; 1];

% Kalman-Filter parameters
S.Q = diag([.1*S.dt, .1*S.dt, .1*S.dt, .1, .1]);
S.R = 1e-2*diag([1e-3, 1e-3, 1, 1]);
S.R_sensor = 1e-4*diag([1, 1, .1, .1]);
P_0 = 1e-3 * diag([1, 1, 1, 1, 1]);

% random number generator
rng(10010); % seeding

%% System Modeling
% set-up data holders
x_true = [x_true0, zeros(5, N-1)];
us = zeros(2,N);

x_kalman = [x_true0, zeros(5, N-1)];
P_kalman = zeros(5,5,N); 
P_kalman(:,:,1) = P_0;

error_x = zeros(5,N);

zs = zeros(4, N);

for i = 1:N-1
   % current state
   t_i = t(i);   
   x_true_i = x_true(:, i);
   x_i = x_kalman(:,i);
   P_i = P_kalman(:,:,i);
   u_i = car_ctrl(t_i, x_true_i, S); % control
%    u_i = S.u0;
   u_i = u_i + sqrt(S.Q(end-1:end, end-1:end))*randn(2,1);
   
   % Update true state (dynamics)
   x_true_ip1 = S.f(x_true_i, u_i, S);
   
   % prediction step
   [x_pred_i, P_pred_i] = ekf_predict(x_i, P_i, u_i, S);
   
   % perform measurement and correction
   z = S.h(x_true_ip1, S) + sqrt(S.R_sensor)*randn(4, 1);
   [x_ip1, P_ip1] = ekf_correct(x_pred_i, P_pred_i, z, S);
   
   % add the data
   x_true(:, i+1) = x_true_ip1;
   x_kalman(:, i+1) = x_ip1;
   P_kalman(:,:, i+1) = P_ip1;
   error_x(:, i+1) = fix_state(x_ip1 - x_true_ip1);
   zs(:, i) = z;
   us(:,i) = u_i;

end

%% Error summarization
error_summ = error_stats(error_x);
disp("Errors:")
disp(error_summ.tbl)

%% Plotting
fig_car = figure(1);
out_vid = VideoWriter(save_base + '_car-traj');
out_vid.FrameRate = 10;
open(out_vid)
for i = 1:10:N
    plot(x_true(1,1:i), x_true(2, 1:i), 'b-', 'DisplayName', 'true'); hold on;
    plot(x_kalman(1, 1:i), x_kalman(2, 1:i), 'g-', 'DisplayName', 'kalman'); hold on;
    plot_car(x_true(:, i), S); hold off;
    axis equal; grid on;
    title("Car trajectory");
    
    writeVideo(out_vid, getframe(fig_car));
    pause(0.05);
end
close(out_vid);
fprintf('Saved video: %s\n', save_base + '_car-traj.' + out_vid.FileFormat);

fig_traj = figure(2);
% trajectory
subplot(2,1,1);
plot(t, x_true(1,:), 'DisplayName', 'p_x: true'); hold on;
plot(t, x_true(2,:), 'DisplayName', 'p_y: true'); hold on;
% plot(t, x_true(3,:), 'DisplayName', '\theta: true'); hold on;
plot(t, x_kalman(1,:), 'DisplayName', 'p_x: kalman'); hold on;
plot(t, x_kalman(2,:), 'DisplayName', 'p_y: kalman'); hold on;
% plot(t, x_kalman(3,:), 'DisplayName', '\theta: kalman'); hold on;
hold off;
xlabel('time'); grid on;
title('position and orientation'); legend('Location', 'bestoutside')

% odometry
subplot(2,1,2);
plot(t, x_true(4,:), 'DisplayName', '\phi: true'); hold on;
plot(t, x_true(5,:), 'DisplayName', 'v: true'); hold on;
plot(t, zs(1,:), 'DisplayName', '\phi: measured'); hold on;
plot(t, zs(2,:), 'DisplayName', 'v: measured'); hold on;
hold off;
xlabel('time'); grid on;
title('odometry: steering angle and velocity'); legend('Location', 'bestoutside')

% error
fig_err = figure(3);
subplot(2,1,1);
plot(t, error_x(1,:), 'DisplayName', 'p_x'); hold on;
plot(t, error_x(2,:), 'DisplayName', 'p_y'); hold on;
hold off;
xlabel('time'); grid on;
title('position error'); legend('Location', 'bestoutside')

subplot(2,1,2);
plot(t, error_x(3,:), 'DisplayName', '\theta'); hold on;
plot(t, error_x(4,:), 'DisplayName', '\phi'); hold on;
hold off;
xlabel('time'); grid on;
title('steering and orientation error'); legend('Location', 'bestoutside');


%% Saving
saveas(fig_car, save_base + '_full-traj.png');
fprintf('Saved figure: %s\n', save_base + '_full-traj.png');

saveas(fig_traj, save_base + '_time-traj.png');
fprintf('Saved figure: %s\n', save_base + '_time-traj.png');

saveas(fig_err, save_base + '_err.png');
fprintf('Saved figure: %s\n', save_base + '_err.png');

save(save_base + ".mat", 'x_kalman', 'error_x', 'P_kalman', 't', 'us', 'x_true',...
    'error_summ', 'S', 'zs');
fprintf('Saved data file: %s\n', save_base + ".mat");

writetable(error_summ.tbl, save_base + '_errors.csv', 'WriteRowNames', true);
fprintf('Saved data table: %s\n', save_base + '_errors.csv');

%% Functions
function error_summ = error_stats(error_x)
    error_summ.mean = mean(abs(error_x), 2);
    error_summ.std = std(abs(error_x), 0, 2);
    error_summ.err = error_x;
    error_summ.tbl = table(error_summ.mean, error_summ.std, 'VariableNames',  {'mean', 'std'},...
        'RowNames', {'px', 'py', 'theta', 'phi', 'v'});
    
end
