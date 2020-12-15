%% car_h.m
%
% car sensor model including:
%   - GPS measurements
%   - odometry model
%
% - written by: Dimitri Lezcano

function [z, varargout] = car_h(x, S)
    [z_odom, H_odom] = car_odom(x, S);
    [z_gps, H_gps] = car_gps(x, S);
    [z_gps2, H_gps2] = car_gps2(x, S);
    [z_pos, H_pos] = car_pos(x, S);
    
    z = [z_odom; z_gps];
    
    % Jacobian
    if nargout > 1
        H = [H_odom; H_gps];
        varargout{1} = H;
        
    end
end

%% Helper functions
% odometry
function [z, varargout] = car_odom(x, S)
    % returns steering wheel angle and speed
    z = [x(4); x(5)];
    
    if nargout > 1
        H = zeros(2,5);
        H(1, 4) = 1;
        H(2, 5) = 1;
        varargout{1} = H;
        
    end
end

% GPS
function [z, varargout] = car_gps(x, S)
    px = x(1); py = x(2);
    d = sqrt(px^2 + py^2);
    z = [atan2( py, px );
         atan2( d, S.Re )];
     
    if nargout > 1
        H = zeros(2,5);
        H = [                   -py/d^2,                   px/d^2, 0, 0, 0;
              S.Re*px/(S.Re^2 + d^2)/d, S.Re*py/(S.Re^2 + d^2)/d, 0, 0, 0];
%         H = [                                  -py/(px^2 + py^2),                                   px/(px^2 + py^2), 0, 0, 0;
%              (S.Re*px)/((px^2 + py^2)^(1/2)*(S.Re^2 + px^2 + py^2)), (S.Re*py)/((px^2 + py^2)^(1/2)*(S.Re^2 + px^2 + py^2)), 0, 0, 0]
          
        varargout{1} = H;
          
    end
end

function [z, varargout] = car_gps2(x, S)
    px = x(1); py = x(2);
    z = [atan2(px, S.Re); atan2(py, S.Re)];
    
    if nargout > 1
%         H = zeros(2,5);
%         H(1,1) = S.Re/(S.Re^2 + px^2);
%         H(2,2) = S.Re/(S.Re^2 + py^2);
        H = [S.Re/(S.Re^2 + px^2), 0, 0, 0, 0;
             0, S.Re/(S.Re^2 + py^2), 0, 0, 0];
        varargout{1} = H;
    end
    
end
    

% direct position modeling
function [z, varargout] = car_pos(x, S)
    z = x(1:2);
    
    if nargout>1
        H = zeros(2,5);
        H(1,1) = 1;
        H(2,2) = 1;
        varargout{1} = H;
    end
end