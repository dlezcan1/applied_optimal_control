%% car_f.m
%
% this models a simple 2-D car model dynamics
%
% - written by: Dimitri Lezcano

function [y, varargout] = car_f(x, u, S)
    % velocity update
    % x = [px, py, theta, delta, v]
    % S.l = the car's wheel base
    v = x(5);
    c = cos(x(3));
    s = sin(x(3));
    t = tan(x(4));
    
    % velocity update
    dx= [v * c;
         v * s;
         v * t/S.l;
         u(1);
         u(2)]; 
     
     y = x + dx * S.dt;
     
     % Jacobian
     if nargout > 1
         F = [1, 0, -v*s, c, 0;
              0, 1,  v*c, s, 0;
              0, 0,    1, t, 1 + t^2;
              0, 0,    0, 1, 0;
              0, 0,    0, 0, 1];
     	 
          varargout{1} = F;
          
     end
end