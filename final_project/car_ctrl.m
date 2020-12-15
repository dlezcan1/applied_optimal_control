%% car_ctrl.m
%
% function to perform car controls
%
% - written by: Dimitri Lezcano

%% main function
function u = car_ctrl(t, x, S)
    if isfield(S, 'w')
        u = car_ctrl_osc(t, x, S);
        
    else
        car_ctrl_const(t, x, S);
    
    end

end


%% Helper functions
% constant control
function u = car_ctrl_const(t, x, S)
    
    u = S.u0 .* ones(2, length(t));
    
end

function u = car_ctrl_osc(t, x, S)
    
    u = S.u0 .* [cos(S.w*t); sin(S.w*t)];
    
end

