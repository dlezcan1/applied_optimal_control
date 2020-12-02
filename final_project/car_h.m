%% car_h.m
%
% car sensor model including:
%   - GPS measurements
%   - odometry model
%
% - written by: Dimitri Lezcano

function [z, varargout] = car_h(x, S)
    z = 0;
    
    % Jacobian
    if nargout > 1
        H = 0;
        varargout{1} = H;
        
    end
    
end