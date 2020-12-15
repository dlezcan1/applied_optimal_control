%% ekf_predict.m
% 
% Extended Kalman filter prediction function
%
% - written by: Dimitri Lezcano

%% main function
function [x,P] = ekf_predict(x, P, u, S)

[x, F] = S.f(x, u, S);
x = fix_state(x, S);  % fix any [-pi,pi] issues
P = F*P*F' + S.Q;

end