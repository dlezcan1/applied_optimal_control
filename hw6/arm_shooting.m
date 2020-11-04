%% arm_shooting.m
%
% this script is to address problem 1 from hw6
%
% a direct shooting optimal control of a 2-link arm
%
% - written by: Dimitri Lezcano

%% Set-up
% model parameters
S.m1 = 1;
S.m2 = 1;
S.l1 = .5;
S.l2 = .5;
S.lc1 = .25;
S.lc2 = .25;
S.I1 = S.m1*S.l1/12;
S.I2 = S.m2*S.l2/12;
S.g = 9.8;

% time horizon and segments
tf = 2; 
S.N = 128;
S.h = tf/S.N; % time step


S.f = @arm_f;

% cost function parameters
S.Q = 0.5 * diag([5, 5, 1, 1]); % for x
S.Qf = diag([10, 10, 1, 1]); 
S.R = 0.01 * eye(2); % for u

S.Qs = sqrt(S.Q);
S.Rs = sqrt(S.R);
S.Qfs = sqrt(S.Qf);

% initial state
x0 = [pi/2; pi/4; -.5; 0]; % initial state
xf = [0; 0; 0; 0]; % final state
xd = repmat(xf, 1, S.N); % desired trajectory

S.x0 = x0;
S.xf = xf;
S.xd = xd;

% initial control sequence
us = zeros(2, S.N);
ud = zeros(2, S.N); % desired control sequence

S.ud = ud;

%% Perform the single direct shooting optimization
lb = repmat([-20;-20], S.N, 1);
ub = -lb;

us = lsqnonlin(@(us) arm_cost(us, S), us, lb, ub);


%% Calculate optimal trajectory and Plot
xs = sys_traj(x0, us, S);

% trajectory plotting
subplot(1,2,1);

plot(xs(1,:), xs(2,:), '-b'); hold on;
plot(xf(1), xf(2), 'r*'); hold off;

y = arm_cost(us, S);
J = y'*y/2;

disp(['cost=' num2str(J)])

xlabel('q1')
ylabel('q2')
title('arm joint trajectory');

% control plotting
subplot(1,2,2)

plot(0:S.h:tf-S.h, us(1,:),0:S.h:tf-S.h, us(2,:));
xlabel('t (sec.)')
legend('u_1','u_2')
title('joint control');


%% Functions
% arm control cost
function y = arm_cost(us, S)
    % used for least squares form
    us = reshape(us, 2, []);
    
    xs = sys_traj(S.x0, us, S); % get system trajectory
    
    N = size(us, 2);
    
    y = [];
    for k = 1:N
        y = [y; S.Rs * (us(:,k) - S.ud(:,k));
                S.Qs * (xs(:,k) - S.xd(:,k))];
            
    end
    
    y = [y; S.Qfs*(xs(:,N+1) - S.xf)];
    
end

% dynamics of arm (taken from arm_sim.m
function [x, A, B] = arm_f(k, x, u, S)
    % arm discrete dynamics
    % set jacobians A, B to [] if unavailable
    %
    % the following parameters should be set:
    % S.m1  : mass of first body
    % S.m2  : mass of second body
    % S.l1  : length of first body
    % S.l2  : length of second body
    % S.lc1 : distance to COM
    % S.lc2 : distance to COM
    % S.I1  : inertia 1
    % S.I2  : inertia 2
    % S.g   : gravity
    %
    % S.h : time-step

    q = x(1:2);
    v = x(3:4);

    c1 = cos(q(1));
    c2 = cos(q(2));
    s2 = sin(q(2));
    c12 = cos(q(1) + q(2));

    % coriolis matrix
    C = -S.m2*S.l1*S.lc2*s2*[v(2), v(1) + v(2);
                        -v(1), 0] + diag([.2;.2]);

    % mass elements
    m11 = S.m1*S.lc1^2 + S.m2*(S.l1^2 + S.lc2^2 + 2*S.l1*S.lc2*c2) + ...
          S.I1 + S.I2;

    m12 = S.m2*(S.lc2^2 + S.l1*S.lc2*c2) + S.I2;

    m22 = S.m2*S.lc2^2 + S.I2;

    % mass matrix
    M = [m11, m12;
         m12, m22];

    % gravity vector
    fg = [(S.m1*S.lc1 + S.m2*S.l1)*S.g*c1 + S.m2*S.lc2*S.g*c12;
          S.m2*S.lc2*S.g*c12];

    % acceleration
    a = inv(M)*(u - C*v - fg);
    v = v + S.h*a;

    x = [q + S.h*v;
         v];

    % leave empty to use finite difference approximation
    A= [];
    B= [];

end

% function to generate the system trajectory
function xs = sys_traj(x0, us, S)

    N = size(us, 2);
    xs(:,1) = x0;

    for k=1:N
      xs(:, k+1) = S.f(k, xs(:,k), us(:,k), S);
    end

end