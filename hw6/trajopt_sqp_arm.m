%% trajopt_sqp_arm.m
%
% this script is to address problem 2 from hw6
%
% a direct collocation optimal control of a 2-link arm
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
S.tf = tf;
S.N = 50;
S.h = tf/S.N; % time step


S.f = @arm_f;
S.L = @arm_L;
S.Lf = @arm_Lf;

% cost function parameters
S.Q = 0.5 * diag([5, 5, 1, 1]); % for x
S.Qf = diag([10, 10, 1, 1]); 
S.R = 0.01 * eye(2); % for u

% initial state
x0 = [pi/2; pi/4; -.5; 0]; % initial state

S.x0 = x0;


% obstacle positions (only circles {center, radius}
S.obstacles = {};
S.obstacles{1} = {[pi/5; pi/5], 0.1};


%% Optimization
% initializations of trajectory
us = zeros(2, S.N);
xs = sys_traj( x0, us, S);

% pack xs and us
z = [reshape(xs(:,2:end), [], 1); reshape(us, [], 1)];

% set-up optimization options and objectives
options = optimset('GradObj','on','GradConstr', 'off', 'MaxIter', ...
                   10000, 'MaxFunEvals', 20000, 'TolCon', 1e-5, 'TolFun', 1e-4, 'TolX', 1e-5);
cost_fn = @ (z) objfun(z, x0, S);
cons = @(z) constraints(z, x0, S, @plottraj);

% perform the optimization
[z, cost, exitflag, output] = fmincon(@(z) cost_fn(z), z, [], [], [], [], [], [], ...
                                      @(z) cons(z), options);
                                  
xs = [x0, reshape(z(1:S.N*4), 4, [])];
us = reshape(z(S.N*4 + 1:end), 2, []);

%% Plotting
global pc
pc = 0; % reset the counter
plottraj(xs, us, S);


%% Functions
% objective function to be optimized
function [f, g] = objfun(z, x0, S)
    % z is a vectorized [x; u] so unpack
    xs = [x0, reshape(z(1:S.N*4), 4, S.N)];
    us = reshape(z(4*S.N + 1:end), 2, S.N);
    
    f = 0;
    for i = 1:S.N+1
        if i < S.N+1
            [L, Lx, Lxx, Lu, Luu] = S.L(i, xs(:,i), us(:,i),S);
            
            % control gradients
            uind = S.N*4 + (i-1)*2;
            g(uind+1:uind + 2) = Lu;
            
        else
            [L, Lx, Lxx] = S.Lf(xs(:, i), S);
            
        end
        
        f = f + L; % add the cost
        
        % set state gradients
        if (i > 1)
            xind = (i - 2) * 4;
            g(xind+1: xind + 4) = Lx;
        end
        
    end
            
end

% implement nonlinear constraints
function [c, ceq] = constraints(z, x0, S, cb)
    % unpack z
    xs = [x0, reshape(z(1:S.N*4), 4, S.N)];
    us = reshape(z(4*S.N + 1:end), 2, S.N);
    
    if ~isempty(cb)
        cb(xs, us, S);
    end
    
    ceq = zeros(4*S.N, 1);
    c = obstacle_contraints(xs, S);
    
    for i = 1:S.N
        % discrete dynamics
        ind = (i - 1) * 4; 
        ceq(ind + 1 : ind + 4) = xs(:, i+1) - S.f(i, xs(:,i), us(:,i), S);
        
    end
        
end

% function for obstacle constraints
function cobs = obstacle_contraints(xs, S)
    cobs = [];
       
    for i = 1:length(S.obstacles)
        % grab the obstacle
        obs_i = S.obstacles{i};

        % calculate distance from circle
        dist = vecnorm(xs(1:2,:) - obs_i{1}, 2, 1) - obs_i{2};
        dist = reshape(dist, [], 1);
        
        % append the constraints
        cobs = [cobs; dist];

    end

end


% arm cost (just standard quadratic cost
function [L, Lx, Lxx, Lu, Luu] = arm_L(k, x, u, S)
    L = S.h/2 * (x' * S.Q * x + u' * S.R * u);
    
    Lx = S.h * S.Q * x;
    Lxx = S.h * S.Q;
    
    Lu = S.h * S.R * u;
    Luu = S.h * S.R;

end

% arm cost (just standard quadratic cost
function [L, Lx, Lxx] = arm_Lf(x, S)


    L = x'*S.Qf*x/2;
    Lx = S.Qf*x;
    Lxx = S.Qf;

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

% the quadratic total cost
function J  = arm_cost(xs, us, S)

    N = size(us, 2);
    J = 0;

    for k=1:N+1
      if k < N+1
        [L, Lx, Lxx, Lu, Luu] = S.L(k, xs(:,k), us(:,k), S);
      else
        [L, Lx, Lxx] = S.Lf(xs(:,end), S);  
      end
      J = J + L;
    end

end

% function to plot the trajectory
function f = plottraj(xs, us, S)

    %only plot every 250'th call
    global pc
    pc = pc + 1;
    if (mod(pc,100))
      return
    end

    subplot(1,2,1)
    
    plot(xs(1,:), xs(2,:), '-b'); hold on;
    
    % draw all obstacles
    for i = 1:length(S.obstacles)
        obs_i = S.obstacles{i};
        viscircles(obs_i{1}', obs_i{2}, 'Color', 'r');
        
    end
    hold off;
    
    xlabel('q1')
    ylabel('q2')
    title('arm joint trajectory');
    axis equal
    drawnow 
    hold on

    subplot(1,2,2)
    plot(0:S.h:S.tf-S.h, us(1,:),0:S.h:S.tf-S.h, us(2,:));
    xlabel('t (sec.)')
    legend('u_1','u_2')
    title('control')

end
