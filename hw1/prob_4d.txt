%% prob_4d.m
%
% This is a script to verify prob 4d's optimization
%
% - written by: Dimitri Lezcano

%% Set-up
% the cost function and the constraints
cost_fn = @(y) y(1).^2 + 4 * y(1) + 8 * y(2).^2;

% constraint A * y <= b | [1 -7]*[x; u] <= -5
A = [1, -7];
b = -5;

x_0 = [0; 1];

%% My optimal solution
y_optim_mine = [-2.4211; 0.3684];

%% using fmincon to get optimized value

y_optim = fmincon(cost_fn, x_0, A, b);

fprintf('   mine: x* = %.5f, u* = %.5f\n', y_optim_mine);
fprintf('fmincon: x* = %.5f, u* = %.5f\n', y_optim);

%% write the output file
file_out = "hw1_prob_4d.txt";
fileID = fopen(file_out, 'w');
fprintf(fileID, '   mine: x* = %.5f, u* = %.5f\n', y_optim_mine);
fprintf(fileID, 'fmincon: x* = %.5f, u* = %.5f\n', y_optim);
fclose(fileID);
disp("Wrote file:" + file_out)