clear; close all; clc;


dt = 0.05;               
t_final = 50;             
time = 0:dt:t_final;      
n_steps = length(time);


start_pos = [-3, 0];     
target_velocity = 0.3;   
end_pos = [10, 0];        


x_obs = [1, 3.5, 3.5, 7];     
y_obs = [0, -1.5, 1.5, 0];    
r_obs = 0.5;               
n_obs = length(x_obs);       


alpha_nom = 1.0;         
kp = 2.0;              
kd = 1.5;                 
u_max = 5.0;          


gp_window_size = 50;    
sigma_f = 0.3;        
sigma_n = 0.05;        
l_p = [3.0, 3.0];        
l_v = [1.5, 1.5];        

gamma_gp = 2.0;          
beta_base = 0.2;         
lambda_vel = 0.3;        


state = [start_pos, 0, 0];


gp_data.X = [];  
gp_data.Y = [];  


trajectory = zeros(n_steps, 4);      
cbf_values = zeros(n_steps, n_obs);  
control_inputs = zeros(n_steps, 2);  
uncertainty_values = zeros(n_steps, 1);  
tracking_error = zeros(n_steps, 1);     
predicted_disturbance = zeros(n_steps, 2); 
actual_disturbance = zeros(n_steps, 2);    
adaptive_radii = zeros(n_steps, n_obs);     



tic;

for k = 1:n_steps
    current_time = time(k);
    pos = state(1:2);
    vel = state(3:4);
    
   
    disturbance_x = 0.2 * sin(2 * pi * pos(1)) + 0.2 * cos(4 * pi * pos(1));
    disturbance_y = 0.2 * sin(1.5 * pi * pos(1) + pi/4);
    disturbance = [disturbance_x, disturbance_y];
    actual_disturbance(k, :) = disturbance;
    
   
    desired_pos = [start_pos(1) + target_velocity * current_time, 0];  % y=0
    desired_vel = [target_velocity, 0];
    
    if desired_pos(1) >= end_pos(1)
        desired_pos = end_pos;
        desired_vel = [0, 0];
    end
    
   
    error_pos = desired_pos - pos;
    error_vel = desired_vel - vel;
    u_nom = kp * error_pos + kd * error_vel;
   
    u_nom(2) = u_nom(2) * 1.5;
    
    tracking_error(k) = abs(pos(2));
    
  
    if size(gp_data.X, 1) >= 5
        [d_pred, sigma_d] = gp_predict(pos, vel, gp_data, sigma_f, sigma_n, l_p, l_v);
        sigma_d = min(max(sigma_d, 0.1), 1.0);
    else
        d_pred = [0, 0];
        sigma_d = 0.3;
    end
    predicted_disturbance(k, :) = d_pred;
    uncertainty_values(k) = sigma_d;
    
    
    [u_safe, h_pbf_values, r_adaptive_values] = gp_pbf_controller(pos, vel, u_nom, d_pred, sigma_d, ...
        x_obs, y_obs, r_obs, gamma_gp, beta_base, lambda_vel, alpha_nom, u_max);
    
    
    trajectory(k, :) = state;
    control_inputs(k, :) = u_safe;
    cbf_values(k, :) = h_pbf_values;
    adaptive_radii(k, :) = r_adaptive_values;
    
 
    if k < n_steps
       
        accel = u_safe + disturbance;
        
       
        if k > 1 && norm(vel) > 0.01
            gp_data = update_gp_data(gp_data, pos, vel, disturbance, gp_window_size);
        end
        
     
        state(3:4) = state(3:4) + accel * dt;
        state(1:2) = state(1:2) + state(3:4) * dt;
    end
    

    if pos(1) >= end_pos(1)
        trajectory = trajectory(1:k, :);
        tracking_error = tracking_error(1:k);
        predicted_disturbance = predicted_disturbance(1:k, :);
        actual_disturbance = actual_disturbance(1:k, :);
        adaptive_radii = adaptive_radii(1:k, :);
        uncertainty_values = uncertainty_values(1:k);
        time = time(1:k);
        break;
    end
end

elapsed_time = toc;







figure;
subplot(2,1,1);
plot(time, actual_disturbance(:,1), 'k-', 'LineWidth', 2, 'DisplayName', '实际扰动');
hold on; grid on;
plot(time, predicted_disturbance(:,1), 'b--', 'LineWidth', 2, 'DisplayName', 'GP预测');
fill([time'; flipud(time')], ...
     [predicted_disturbance(:,1) + uncertainty_values; ...
      flipud(predicted_disturbance(:,1) - uncertainty_values)], ...
     'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '不确定性边界');
xlabel('时间 (s)', 'FontSize', 12);
ylabel('X (m/s²)', 'FontSize', 12);
legend('Location', 'best');

subplot(2,1,2);
plot(time, actual_disturbance(:,2), 'k-', 'LineWidth', 2, 'DisplayName', '实际扰动');
hold on; grid on;
plot(time, predicted_disturbance(:,2), 'r--', 'LineWidth', 2, 'DisplayName', 'GP预测');
fill([time'; flipud(time')], ...
     [predicted_disturbance(:,2) + uncertainty_values; ...
      flipud(predicted_disturbance(:,2) - uncertainty_values)], ...
     'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '不确定性边界');
xlabel('时间 (s)', 'FontSize', 12);
ylabel('Y (m/s²)', 'FontSize', 12);
legend('Location', 'best');


figure;
colors = lines(n_obs);
hold on; grid on;

for i = 1:n_obs
    plot(time, adaptive_radii(:, i), '-', 'Color', colors(i,:), ...
         'LineWidth', 2, 'DisplayName', sprintf('障碍物%d', i));
end


plot([0, time(end)], [r_obs, r_obs], 'k--', 'LineWidth', 2, 'DisplayName', '原始半径');

xlabel('时间 (s)', 'FontSize', 12);
ylabel('安全半径 (m)', 'FontSize', 12);
legend('Location', 'best');



function [u_safe, h_pbf_values, r_adaptive_values] = gp_pbf_controller(pos, vel, u_nom, d_pred, sigma_d, ...
    x_obs, y_obs, r_obs, gamma_gp, beta_base, lambda_vel, alpha_nom, u_max)
    
    pos = pos(:);
    vel = vel(:);
    u_nom = u_nom(:);
    d_pred = d_pred(:);
    
    n_obs = length(x_obs);
    h_pbf_values = zeros(n_obs, 1);
    r_adaptive_values = zeros(n_obs, 1);
    
    
    H = eye(2);
    f = -u_nom;
    
    A = [];
    b = [];
    
    for i = 1:n_obs
        obs_center = [x_obs(i); y_obs(i)];
        
       
        dist_to_obs = norm(pos - obs_center);
        
       
        vel_factor = norm(vel) / (dist_to_obs + 0.1);
        beta_i = beta_base + gamma_gp * vel_factor;
        r_adaptive = r_obs + beta_i * sigma_d;
        r_adaptive_values(i) = r_adaptive;
        
      
        h_pbf = dist_to_obs^2 - r_adaptive^2;
        h_pbf_values(i) = h_pbf;
        
        
        if dist_to_obs < r_adaptive + 1.0
            % 梯度
            grad_h = 2 * (pos - obs_center);
            
            
            alpha_i = alpha_nom * (1 + lambda_vel * norm(vel));
            
          
            Lfh = grad_h' * vel;
            Lgh = grad_h';
            
         
            disturbance_compensation = -gamma_gp * norm(grad_h) * sigma_d;
            
            A = [A; -Lgh];
            b = [b; Lfh + alpha_i * h_pbf + disturbance_compensation];
        end
    end
    
    A_bounds = [eye(2); -eye(2)];
    b_bounds = [u_max * ones(2, 1); u_max * ones(2, 1)];
    
    A = [A; A_bounds];
    b = [b; b_bounds];
    
  
    if size(A, 1) == 4  
        u_safe = u_nom;
        u_safe = max(-u_max, min(u_max, u_safe));
    else
        try
            options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
            [u_safe, ~, exitflag] = quadprog(H, f, A, b, [], [], [], [], u_nom, options);
            
            if exitflag < 0 || isempty(u_safe)
                u_safe = u_nom;
                for i = 1:n_obs
                    obs_center = [x_obs(i); y_obs(i)];
                    dist = norm(pos - obs_center);
                    if dist < r_obs + 0.5
                        repel_dir = (pos - obs_center) / dist;
                        u_safe = u_safe + 2.0 * repel_dir;
                    end
                end
            end
        catch
            u_safe = u_nom;
        end
    end
    
    u_safe = u_safe(:)';
end


function [mu_d, sigma_d] = gp_predict(pos, vel, gp_data, sigma_f, sigma_n, l_p, l_v)
    
    if isempty(gp_data.X) || size(gp_data.X, 1) < 3
        mu_d = [0, 0];
        sigma_d = sigma_f;
        return;
    end
    
    x_star = [pos(:); vel(:)];
    
    n_use = min(size(gp_data.X, 1), 30);
    X_use = gp_data.X(end-n_use+1:end, :);
    Y_use = gp_data.Y(end-n_use+1:end, :);
    
    n = size(X_use, 1);
    K = zeros(n, n);
    k_star = zeros(n, 1);
    
    for i = 1:n
        for j = i:n
            k_ij = se_kernel(X_use(i,:)', X_use(j,:)', sigma_f, sigma_n, l_p, l_v, i==j);
            K(i,j) = k_ij;
            K(j,i) = k_ij;
        end
        k_star(i) = se_kernel(X_use(i,:)', x_star, sigma_f, 0, l_p, l_v, false);
    end
    
    try
        K_reg = K + (sigma_n^2 + 1e-4) * eye(n);
        L = chol(K_reg, 'lower');
        
        alpha = L' \ (L \ Y_use);
        mu_d = (k_star' * alpha)';
        
        v = L \ k_star;
        k_star_star = sigma_f^2;
        var_d = k_star_star - v' * v;
        sigma_d = sqrt(max(var_d, 0.01));
        
    catch
        mu_d = [0, 0];
        sigma_d = sigma_f;
    end
end


function k = se_kernel(x1, x2, sigma_f, sigma_n, l_p, l_v, is_same)
    p1 = x1(1:2);
    v1 = x1(3:4);
    p2 = x2(1:2);
    v2 = x2(3:4);
    
    dist_p = sum(((p1 - p2) ./ l_p(:)).^2);
    dist_v = sum(((v1 - v2) ./ l_v(:)).^2);
    
    k = sigma_f^2 * exp(-0.5 * (dist_p + dist_v));
    
    if is_same
        k = k + sigma_n^2;
    end
end


function gp_data = update_gp_data(gp_data, pos, vel, disturbance, window_size)
    
    new_x = [pos(:); vel(:)]';
    new_y = disturbance(:)';
    
    if isempty(gp_data.X)
        gp_data.X = new_x;
        gp_data.Y = new_y;
    else
        gp_data.X = [gp_data.X; new_x];
        gp_data.Y = [gp_data.Y; new_y];
    end
    
    if size(gp_data.X, 1) > window_size
        gp_data.X = gp_data.X(end-window_size+1:end, :);
        gp_data.Y = gp_data.Y(end-window_size+1:end, :);
    end
end