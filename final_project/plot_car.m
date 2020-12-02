%% plot_car.m
%
% function to plot the car

function plot_car(x, S) 
    % grab the state variables
    p = x(1:2); % position
    th = x(3); % theta
    d = x(4); % delta
    
    % helpful points
    u = [cos(th); sin(th)]; % unit vector
    u_perp = [-sin(th); cos(th)]; % perpendicular
    u_frontwb = [-sin(th+d); cos(th+d)];
    u_frontwhl = [cos(th + d); sin(th+d)];
    
    %% key points of the car
    front = S.l*u + p; % front of the car
    
    % wheel base
    back_wb1 = p - S.l/4 * u_perp;
    back_wb2 = p + S.l/4 * u_perp;
    
    front_wb1 = front - S.l/4 * u_frontwb;
    front_wb2 = front + S.l/4 * u_frontwb;
    
    
    
    % lines of the car
    lnb = [p, front];
    ln_bwb = [back_wb1, back_wb2];
    ln_fwb = [front_wb1, front_wb2];
    br_whl = [back_wb1 - S.l/8 * u, back_wb1 + S.l/8 * u];
    bl_whl = [back_wb2 - S.l/8 * u, back_wb2 + S.l/8 * u];
    fr_whl = [front_wb1 - S.l/8 * u_frontwhl, front_wb1 + S.l/8 * u_frontwhl];
    fl_whl = [front_wb2 - S.l/8 * u_frontwhl, front_wb2 + S.l/8 * u_frontwhl];
    
    % plot the car
    plot(lnb(1,:), lnb(2,:), 'k-'); hold on;
    plot(ln_bwb(1,:), ln_bwb(2,:), 'k-'); hold on;
    plot(ln_fwb(1,:), ln_fwb(2,:), 'k-'); hold on;
    plot(bl_whl(1,:), bl_whl(2,:), 'k-'); hold on;
    plot(br_whl(1,:), br_whl(2,:), 'k-'); hold on;
    plot(fr_whl(1,:), fr_whl(2,:), 'k-'); hold on;
    plot(fl_whl(1,:), fl_whl(2,:), 'k-'); hold off;
    
    

end