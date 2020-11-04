%% prob_3.m
%
% this script is to solve problem 3's part of solving the Ricatti equation for
% optimal control
%
% - written by: Dimitri Lezcano

%% Set-up system params
global A B R Q Pf Rinv

% system parameters
A = [0 1; 2 -1];
B = [0; 1];
R = 1;
Rinv = inv(R);
Q = diag([2, 1]);
Pf = zeros(2);
tf = 20;

%% calculate P(t)
tspan_P = [tf, 0];
[tP, Pv] = ode45(@(t, Pv) riccati(t, Pv), tspan_P, reshape(Pf, [], 1));

Pv = Pv'; % transpose Pv to be 4 x N
P = reshape(Pv, 2, 2, []);

%% calculate the dynamics
tspan = [0, tf];
x_0 = [-5; 5];
[tx, x] = ode45(@(t, x) dynamics(t, x, P, tP), tspan, x_0);

x = x'; % transpose it to be 2 x N

%% Calculate the control
u = zeros(1, length(tx));
for i = 1:length(x)
    t_i = tx(i); % time at this instance
    x_i = x(:,i);
    
    % calculate the control 
    u(i) = control_law(t_i, x_i, P, tP);
    
end

%% Plotting
fig = figure(1);

% plot P(t)
subplot(3,1,1);
plot(tP, Pv); 
xlabel('t'); ylabel('element of P');
legend('P_{11}', 'P_{21}', 'P_{12}', 'P_{22}');
title('P(t)');

% plot x(t)
subplot(3,1,2);
plot(tx, x);
xlabel('t'); ylabel('x_i');
legend('x_1', 'x_2');
title('x(t)');

% plot u(t)
subplot(3,1,3);
plot(tx, u);
xlabel('t'); ylabel('u');
title('u(t)');

%% Saving the figure
fig_save = 'prob_3.jpg';
saveas(fig, fig_save);
fprintf('Saved figure: %s\n\n', fig_save);

%% Functions
% riccati differential equation

function dPv = riccati(t, Pv)
    global A B Q Rinv
    % turn P into a matrix again
    P = reshape(Pv, 2,2);
    
    % calculate matrix dP
    dP = -A'* P - P * A + P * B * Rinv * B' * P - Q;
    
    % vectorize dP
    dPv = reshape(dP, [], 1);    
    
end

% function for computing the dynamics
function dx = dynamics(t, x, P, tP)
    % P is of shape 2x2xN : N is the number of time elements
    global A B
    u = control_law(t, x, P, tP);
   
    dx = A * x + B * u;
   

end

function u = control_law(t, x, P, tP)
    global Rinv B
    [~, t_idx] = min(abs(t - tP));
    u = -Rinv * B' * P(:,:,t_idx) * x;
    
    
end