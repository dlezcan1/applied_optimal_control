%% car_ctrl.m
%
% function to perform car controls
%
% - written by: Dimitri Lezcano

%% main function
function u = car_ctrl(t, x, S)
    


end


%% Helper functions
% constant control
function u = car_ctrl_const(t, x, S)
    
    u = S.u0 .* ones(2, length(t));
    
end

function u = car_ctrl_osc(t, x, S)
    
    u = [S.ua; S.ud

