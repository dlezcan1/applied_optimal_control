%% fix_state.m
%
% function to ensure angles are w/in fixed range [-pi, pi]
%
% - written by: Dimitri Lezcano

function x = fix_state(x, S)
    x(3) = fix_angle(x(3));
    x(4) = fix_angle(x(4));
end
    