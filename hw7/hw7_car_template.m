clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'hw7_car');
    
    % (a) Define states and controls 

    
    %% (a) Differential Equation

    
    %% (b) Optimal Control Problem
    
                      
    % (b) Minimize control effort

    
    % (c) Path constraints
                  
    
    % (d) Initial Conditions

    
    % (d) Final boundary conditions
    
    
    %% (e) Optimization Algorithm

    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = hw7_car_RUN();