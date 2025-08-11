%%  Obstacle Avoidance Simulation Based on Robust CBF - 100 Monte Carlo Experiments

clear; close all; clc;



n_trials = 100;
dt = 0.05;                
t_final = 50;             
time = 0:dt:t_final;      
n_steps = length(time);
start_x = -3;            
start_y_range = [-2, 2];  
target_velocity = 0.3;   
end_pos = [10, 0];   
x_obs = [1, 3.5, 3.5, 7];   
y_obs = [0, -1.5, 1.5, 0];   
r_obs = 0.5;                 
n_obs = length(x_obs);       

alpha = 1.0;             
kp = 1.0;                
kd = 1.0;                
u_max = 5.0;             


d_max = 0.2;             
epsilon = 0.01;           


all_trajectories = cell(n_trials, 1);  
all_cbf_values = cell(n_trials, 1);    
all_safety_violations = zeros(n_trials, 1); 
all_start_positions = zeros(n_trials, 2); 


fprintf('Start 100 Monte Carlo simulations (Robust CBF)...\n');
tic;

for trial = 1:n_trials
    if mod(trial, 10) == 0
        fprintf('Complete the %d -th simulation\n', trial);
    end
    

    start_y = start_y_range(1) + (start_y_range(2) - start_y_range(1)) * rand();
    start_pos = [start_x, start_y];
    all_start_positions(trial, :) = start_pos;
    

    state = [start_pos, 0, 0];
    

    trajectory = zeros(n_steps, 4);
    cbf_values = zeros(n_steps, n_obs);
    control_inputs = zeros(n_steps, 2);
    safety_violations = 0;
    
  
    for k = 1:n_steps
        current_time = time(k);
        pos = state(1:2);
        vel = state(3:4);        
        disturbance_x = 0.2 * sin(2 * pi * pos(1)) + 0.2 * cos(4 * pi * pos(1));
        disturbance_y = 0.2 * sin(1.5 * pi * pos(1) + pi/4);
        disturbance = [disturbance_x, disturbance_y];     
     
        desired_pos = [start_pos(1) + target_velocity * current_time, start_pos(2)];
        desired_vel = [target_velocity, 0];        
       
        if desired_pos(1) >= end_pos(1)
            desired_pos = end_pos;
            desired_vel = [0, 0];
        end       
     
        u_nom = kp * (desired_pos - pos) + kd * (desired_vel - vel);        
        u_safe = robust_cbf_controller(pos, vel, u_nom, x_obs, y_obs, r_obs, alpha, u_max, d_max, epsilon);        
        trajectory(k, :) = state;
        control_inputs(k, :) = u_safe;
        for i = 1:n_obs
            obs_center = [x_obs(i), y_obs(i)];
            h = sum((pos - obs_center).^2) - r_obs^2;
            cbf_values(k, i) = h;
            
            if h < 0
                safety_violations = safety_violations + 1;
            end
        end
        
        if k < n_steps
            accel = u_safe + disturbance;
            
            state(3:4) = state(3:4) + accel * dt;
            state(1:2) = state(1:2) + state(3:4) * dt;
        end
        
        if pos(1) >= end_pos(1)
            trajectory = trajectory(1:k, :);
            cbf_values = cbf_values(1:k, :);
            break;
        end
    end
    
    all_trajectories{trial} = trajectory;
    all_cbf_values{trial} = cbf_values;
    all_safety_violations(trial) = safety_violations;
end

elapsed_time = toc;
fprintf('Simulation completed! Total time used: %.2f seconds\n', elapsed_time);

%% 
figure;
hold on; grid on;

trajectory_color = [0.20, 0.29, 0.37];
for trial = 1:n_trials
    traj = all_trajectories{trial};
    plot(traj(:, 1), traj(:, 2), 'Color', trajectory_color, 'LineWidth', 0.8);
end


start_positions = all_start_positions;
plot(start_positions(:, 1), start_positions(:, 2), 'o', 'MarkerSize', 4, ...
     'MarkerFaceColor', [0.30, 0.69, 0.31], 'MarkerEdgeColor', [0.30, 0.69, 0.31], 'LineWidth', 1);


for i = 1:n_obs
    theta = 0:0.1:2*pi;
    if i == 1 
        x_circle = x_obs(i) + 0.0 + r_obs * cos(theta);
        y_circle = y_obs(i) + r_obs * sin(theta);
    elseif i == 4            
        x_circle = x_obs(i) + 0.0 + r_obs * cos(theta);
        y_circle = y_obs(i) + r_obs * sin(theta);
    else
        x_circle = x_obs(i) + r_obs * cos(theta);
        y_circle = y_obs(i) + r_obs * sin(theta);
    end
    fill(x_circle, y_circle, 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'red', 'LineWidth', 2);
end


plot(end_pos(1), end_pos(2), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'red', 'LineWidth', 2);


xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robust CBF Obstacle Avoidance Trajectory - 100 Monte Carlo Simulations', 'FontSize', 14, 'FontWeight', 'bold');
legend;
axis equal;


text(start_x, max(start_y_range) + 0.3, 'Start area', 'FontSize', 12, 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');
text(end_pos(1), end_pos(2) + 0.3, 'End', 'FontSize', 12, 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');

% 标注障碍物
for i = 1:n_obs
    text(x_obs(i), y_obs(i) + r_obs + 0.2, sprintf('Obstacle%d', i), ...
         'FontSize', 10, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end


figure;
hold on; grid on;


for trial = 1:n_trials
    cbf_vals = all_cbf_values{trial};
    if size(cbf_vals, 1) > 1
        min_cbf_per_time = min(cbf_vals, [], 2);
        t_vals = linspace(0, size(cbf_vals, 1) * dt, size(cbf_vals, 1));
        plot(t_vals, min_cbf_per_time, 'Color', trajectory_color, 'LineWidth', 0.8);
    end
end


ylims = ylim;
plot([0, t_final], [0, 0], 'r--', 'LineWidth', 3, 'DisplayName', 'Safety boundary (h=0)');

fill([0, t_final, t_final, 0], [ylims(1), ylims(1), 0, 0], ...
     'red', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Unsafe Region');

xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Minimum CBF value h_{min}(x)', 'FontSize', 12, 'FontWeight', 'bold');
% title('无人机与所有障碍物的最小CBF值变化 (Robust CBF) - 100次仿真', 'FontSize', 14, 'FontWeight', 'bold');

ylim([min(-3, ylims(1)), max(15, ylims(2))]);
xlim([0, t_final]);
legend('Location', 'best');

%% 统计分析
figure;
histogram(all_safety_violations, 'BinWidth', 1, 'FaceColor', 'blue');
xlabel('Number of safety violations');
ylabel('Number of simulations');
title('Distribution of number of safety violations (Robust CBF)');
grid on;

mean_violations = mean(all_safety_violations);
text(0.7, 0.9, sprintf('Average violations: %.2f', mean_violations), 'Units', 'normalized', ...
     'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', 'white');

fprintf('\n=== Robust CBF Simulation Result Statistical Summary ===\n');
fprintf('Total number of simulations: %d\n', n_trials);
fprintf('Starting position range: x=%.1f, y∈[%.1f, %.1f]\n', start_x, start_y_range(1), start_y_range(2));
fprintf('Average starting Y coordinate: %.3f\n', mean(all_start_positions(:, 2)));
fprintf('Standard deviation of starting Y coordinate: %.3f\n', std(all_start_positions(:, 2)));
fprintf('Average number of safety violations: %.2f\n', mean_violations);
fprintf('Safety violation rate: %.1f%%\n', sum(all_safety_violations > 0) / n_trials * 100);
fprintf('Maximum disturbance bound: %.2f\n', d_max);
fprintf('Robustness margin: %.2f\n', epsilon)




function u_safe = robust_cbf_controller(pos, vel, u_nom, x_obs, y_obs, r_obs, alpha, u_max, d_max, epsilon)
    pos = pos(:);
    vel = vel(:);
    u_nom = u_nom(:);
    
    n_obs = length(x_obs);

    H = eye(2);
    f = -u_nom;
    
    A = [];
    b = [];
    
    for i = 1:n_obs
        obs_center = [x_obs(i); y_obs(i)];        
        h = norm(pos - obs_center)^2 - r_obs^2;        
        grad_h = 2 * (pos - obs_center);
        
        % Robust CBF约束: Lfh + Lgh*u + alpha*h >= d_max * ||grad_h|| + epsilon
        Lfh = grad_h' * vel;
        Lgh = grad_h';        
        robustness_term = d_max * norm(grad_h) + epsilon;
        
        A = [A; -Lgh];
        b = [b; Lfh + alpha * h - robustness_term];
    end
    

    lb = -u_max * ones(2, 1);
    ub = u_max * ones(2, 1);
    

    if isempty(A)
        u_safe = u_nom;
    else
        try
            options = optimoptions('quadprog', 'Display', 'off');
            u_safe = quadprog(H, f, A, b, [], [], lb, ub, [], options);
        end
    end
    

    u_safe = max(lb, min(ub, u_safe));
    

    u_safe = u_safe';
end

function u_fallback = compute_fallback_control(pos, vel, x_obs, y_obs, r_obs, u_max)
  
    
    n_obs = length(x_obs);
    repulsive_force = zeros(2, 1);
    
    for i = 1:n_obs
        obs_center = [x_obs(i); y_obs(i)];
        dist_to_obs = norm(pos - obs_center);
        
        if dist_to_obs < 2 * r_obs 
            
            direction = (pos - obs_center) / dist_to_obs;
            magnitude = u_max * (2 * r_obs - dist_to_obs) / r_obs;
            repulsive_force = repulsive_force + magnitude * direction;
        end
    end
    

    if norm(repulsive_force) > u_max
        u_fallback = u_max * repulsive_force / norm(repulsive_force);
    else
        u_fallback = repulsive_force;
    end
end