%% car_f.m
%
% this models a simple 2-D car model dynamics
%
% - written by: Dimitri Lezcano

function [y, varargout] = car_f(x, u, S)
    % velocity update
    % x = [px, py, theta, phi, v]
    % S.l = the car's wheel base
    v = x(5);
    cth = cos(x(3));
    sth = sin(x(3));
    cphi = cos(x(4));
    sphi = sin(x(4));
    
    % velocity update
    dx= [v * cth * cphi;
         v * sth * cphi;
         v * sphi/S.l;
         u(1);
         u(2)]; 
     
     y = x + dx * S.dt;
     
     % Jacobian
     if nargout > 1
         %    x, y,          th,         phi, v
         F = [1, 0, -v*sth*cphi, -v*cth*sphi, cth*cphi;
              0, 1,  v*cth*cphi, -v*sth*sphi, sth*sphi;
              0, 0,           1,           t, sphi;
              0, 0,           0,           1, 0;
              0, 0,           0,           0, 1];
     	 
          varargout{1} = F;
          
     end
end