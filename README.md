# MPC-rover
%% MATLAB 2D Rover Simulation - Pure Pursuit vs MPC
% Simulates a rover following a predefined path using Pure Pursuit Control (PPC) and Model Predictive Control (MPC).

clc; clear; close all;

%% Define Environment and Path
T = 0.1; % Time step (s)
steps = 200; % Simulation steps
L = 1.5; % Rover wheelbase (m)
v = 1.0; % Constant velocity (m/s)

% Define predefined path (straight line with slight curve)
path = [linspace(0, 20, 50)', sin(linspace(0, 2*pi, 50))' * 2 + 5];

% Define obstacles (simple static points)
obstacles = [5, 6; 10, 4; 15, 7];

%% Simulate Pure Pursuit Control
fprintf("Running Pure Pursuit Simulation...\n");
pure_pursuit_simulation(path, L, v, T, steps, obstacles);

%% Simulate Model Predictive Control
fprintf("Running Model Predictive Control Simulation...\n");
mpc_simulation(path, L, v, T, steps, obstacles);

%% Function Definitions

function pure_pursuit_simulation(path, L, v, T, steps, obstacles)
    pos = path(1, :);
    theta = 0;
    lookahead_dist = 2.0;
    
    figure; hold on; grid on;
    plot(path(:,1), path(:,2), 'b--', 'LineWidth', 1.5);
    scatter(obstacles(:,1), obstacles(:,2), 100, 'k', 'filled');
    
    for i = 1:steps
        target_idx = find_lookahead_point(pos, path, lookahead_dist);
        if isempty(target_idx), break; end
        
        alpha = atan2(path(target_idx,2) - pos(2), path(target_idx,1) - pos(1)) - theta;
        delta = atan2(2 * L * sin(alpha) / lookahead_dist, 1);
        
        theta = theta + (v/L) * tan(delta) * T;
        pos = pos + v * [cos(theta), sin(theta)] * T;
        
        scatter(pos(1), pos(2), 'ro'); pause(0.05);
    end
    title('Pure Pursuit Control');
end

function mpc_simulation(path, L, v, T, steps, obstacles)
    pos = path(1, :);
    theta = 0;
    
    mpc_obj = mpc(1, 10);
    mpc_obj.Model.Plant = ss([0 1; 0 0], [0; 1], [1 0], 0);
    mpc_obj.Weights.ManipulatedVariableRate = 0.1;
    mpc_obj.Weights.Output = 1;
    
    figure; hold on; grid on;
    plot(path(:,1), path(:,2), 'b--', 'LineWidth', 1.5);
    scatter(obstacles(:,1), obstacles(:,2), 100, 'k', 'filled');
    
    for i = 1:steps
        target_idx = find_lookahead_point(pos, path, 2.0);
        if isempty(target_idx), break; end
        
        u = mpcmove(mpc_obj, [theta; 0], path(target_idx, :));
        delta = u(1);
        
        theta = theta + (v/L) * tan(delta) * T;
        pos = pos + v * [cos(theta), sin(theta)] * T;
        
        scatter(pos(1), pos(2), 'ro'); pause(0.05);
    end
    title('Model Predictive Control');
end

function target_idx = find_lookahead_point(pos, path, lookahead_dist)
    dists = vecnorm(path - pos, 2, 2);
    target_idx = find(dists > lookahead_dist, 1, 'first');
end
