%% prob_2.m
%
% script for problem 2 of homework 7
%
% - written by: Dimitri Lezcano

clear;

%% Set-Up 
t_0 = 0;
num_steps = 64;
obstacles = {{[-7; 0], 1}}; % center and radius

%% Initial Optimization
disp('Initial Optimization');
out = hw7_car_RUN();
u_0 = out.CONTROLS; % initial controls
clc;

%% ACADO optimization
disp('Beginning actual optimization');

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'hw7_car2');
    
    % (a) Define states and controls 
    Parameter T;  % final time
    DifferentialState px;
    DifferentialState py;
    DifferentialState theta;
    DifferentialState delta;
    DifferentialState v;
    Control u_a;
    Control u_delta;
    
    
    %% (a) Differential Equation
    f = acado.DifferentialEquation(0, T);
    f.add( dot(px) == v * cos(theta) );
    f.add( dot(py) == v * sin(theta) );
    f.add( dot(theta) == v * tan(delta) );
    f.add( dot(v) == u_a );
    f.add( dot(delta) == u_delta );
    
    %% (b) Optimal Control )
    ocp = acado.OCP(0.0, T, num_steps);
                      
    % (b) Minimize control effort
    ocp.minimizeLSQ( u_delta, 0 )
    ocp.minimizeMayerTerm(T);
    
    % (c) Path constraints
    ocp.subjectTo( f ); % dynamics
    ocp.subjectTo( -5 <= v <= 5 );
    ocp.subjectTo( -5 <= u_a <= 5 );
    ocp.subjectTo( -pi/4 <= delta <= pi/4 );
    ocp.subjectTo( -pi/6 <= u_delta <= pi/6 );
   
    for i = 1:length(obstacles) % add obstacles
        obs = obstacles{i};
        pc = obs{1};
        r = obs{2};
        
        ocp.subjectTo( r^2 <= (px - pc(1))^2 + (py - pc(2))^2);
        
    end
    
    % (d) Initial Conditions
    ocp.subjectTo( 'AT_START', px == -10.0 );
    ocp.subjectTo( 'AT_START', py == 1.0 );
    ocp.subjectTo( 'AT_START', v == 0.0 );
    ocp.subjectTo( 'AT_START', theta == 0.0 );
    ocp.subjectTo( 'AT_START', delta == 0.0 );
    
    % (d) Final boundary conditions
    ocp.subjectTo( 'AT_END', px == 0.0 );
    ocp.subjectTo( 'AT_END', py == 0.0 );
    ocp.subjectTo( 'AT_END', v == 0.0 );
    ocp.subjectTo( 'AT_END', theta == 0.0 );
    ocp.subjectTo( 3 <= T <= 10 );
    
    
    %% (e) Optimization Algorithm
    algo = acado.OptimizationAlgorithm( ocp );
    
    % algorithm parameters
    algo.set( 'KKT_TOLERANCE', 1e-8 );  
    algo.set( 'DISCRETIZATION_TYPE', 'MULTIPLE_SHOOTING' );
    algo.set( 'MAX_NUM_ITERATIONS', 500 );
    
    % initializations
    algo.initializeControls(u_0);
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = hw7_car2_RUN();

%% Plot
x = out.STATES;
u = out.CONTROLS;

f = figure(1);
% car trajectory
subplot(1,2,1);

for i=1:length(obstacles)
    a = 0: 0.1 : 2*pi;
    % draw obstacle
    obs = obstacles{1};
    pc = obs{1}; 
    r = obs{2};
    plot(pc(1) + cos(a)*r,  pc(2) + sin(a)*r, '-r','LineWidth',2); hold on;
 end

plot(x(1,2), x(1,3), 'r*', 'DisplayName', 'start'); hold on;
plot(x(:,2), x(:,3), 'DisplayName', 'trajectory'); hold on;
plot(x(end,2), x(end,3), 'g*', 'DisplayName', 'end'); hold off;
grid on; legend();
xlabel('p_x'); ylabel('p_y');
title('Trajectory');
axis equal;

subplot(1,2,2);
plot(u(:,1), u(:,2), 'DisplayName', 'u_a'); hold on;
plot(u(:,1), u(:,3), 'DisplayName', 'u_\delta'); hold off;
grid on; legend()
xlabel('u_a'); ylabel('u');
title('Controls');

%% Saving
saveas(f, 'prob_2_results.png');
fprintf('Saved figure: %s\n\n', 'prob_2_results.png');



