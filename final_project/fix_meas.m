%% fix_state.m
%
% function to ensure angles are w/in fixed range [-pi, pi]
%
% - written by: Dimitri Lezcano

function z = fix_state(z, S)
    z(1) = fix_angle(z(1)); % phi
    z(3) = fix_angle(z(3)); % latitude
    z(4) = fix_angle(z(4)); % longitude
end
    