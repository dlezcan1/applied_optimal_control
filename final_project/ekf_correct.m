%% ekf_predict.m
% 
% Extended Kalman filter prediction function
%
% - written by: Dimitri Lezcano

%% main function
function [x,P] = ekf_correct(x, P, z, S)

[y, H] = S.h(x, S);
P = P - P*H'*inv(H*P*H' + S.R)*H*P;
K = P*H'*inv(S.R);

e = z - y;
e = fix_meas(e, S);  % fix any [-pi,pi] issues
x = x + K*e;

end