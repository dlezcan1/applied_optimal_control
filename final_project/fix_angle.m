%% fix_angle.m
% 
% function to keep angle between [-pi, pi]
%
% - written by: Dimitri Lezcano

function theta = fix_angle(theta)
    theta = mod(theta,2*pi);
    
    if theta < -pi
      theta = theta + 2*pi;
    
    elseif theta > pi
        theta = theta - 2*pi;
      
    end
    
end