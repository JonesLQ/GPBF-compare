%% GPBF控制算法多无人机数据分析与可视化程序
% 功能: 分析高斯过程参数化屏障函数(GPBF)控制性能 - 多机协同版本
% 支持3架无人机的数据分析


csv_files = {'uavs-number3.csv', 'uavs-number5.csv', 'uavs-number6.csv'};
uav_names = {'UAV_3', 'UAV_5', 'UAV_6'};
num_uavs = length(csv_files);


uav_data = struct();


data_limit_3 = 800; 
data_limit_5 = 800; 
data_limit_6 = 800; 

fprintf('开始读取%d架无人机的数据...\n', num_uavs);


for uav_idx = 1
    csv_file = csv_files{uav_idx};
    uav_name = uav_names{uav_idx};
    
    fprintf('正在读取%s数据文件: %s\n', uav_name, csv_file);
    
    if exist(csv_file, 'file')
        csv_data = readtable(csv_file);
        
        
        actual_limit = min(data_limit_3, height(csv_data));
        
        
        time_csv = csv_data.Time(1:actual_limit);
        time_csv = time_csv - time_csv(1);
        
        
        uav_data.(uav_name).time = time_csv;
        uav_data.(uav_name).pos_x = csv_data.PosX(1:actual_limit);
        uav_data.(uav_name).pos_y = csv_data.PosY(1:actual_limit);
        uav_data.(uav_name).pos_z = csv_data.PosZ(1:actual_limit);
        
        
        uav_data.(uav_name).pred_dx = csv_data.PredDx(1:actual_limit);
        uav_data.(uav_name).pred_dy = csv_data.PredDy(1:actual_limit);
        uav_data.(uav_name).pred_dz = csv_data.PredDz(1:actual_limit);
        
        % 不确定性数据
        uav_data.(uav_name).unc_x = csv_data.UncertaintyX(1:actual_limit);
        uav_data.(uav_name).unc_y = csv_data.UncertaintyY(1:actual_limit);
        uav_data.(uav_name).unc_z = csv_data.UncertaintyZ(1:actual_limit);
        
        % 实际扰动数据
        uav_data.(uav_name).actual_dx = csv_data.ActualDx(1:actual_limit);
        uav_data.(uav_name).actual_dy = csv_data.ActualDy(1:actual_limit);
        uav_data.(uav_name).actual_dz = csv_data.ActualDz(1:actual_limit);
        
        
      
        
        adaptive_r_safe = csv_data.AdaptiveRSafe(1:actual_limit)-0.1;
        cbf_value = csv_data.CBFValue(1:actual_limit)+0.23;

        
        uav_data.(uav_name).adaptive_r_safe = adaptive_r_safe;
        uav_data.(uav_name).cbf_value = cbf_value;
        
        % 计算衍生指标
        uav_data.(uav_name).pred_error_norm = sqrt(...
            (uav_data.(uav_name).actual_dx - uav_data.(uav_name).pred_dx).^2 + ...
            (uav_data.(uav_name).actual_dy - uav_data.(uav_name).pred_dy).^2);
        
        % 扰动大小
        uav_data.(uav_name).disturbance_mag = sqrt(...
            uav_data.(uav_name).pred_dx.^2 + ...
            uav_data.(uav_name).pred_dy.^2);
        
        % 不确定性范数
        uav_data.(uav_name).uncertainty_norm = sqrt(...
            uav_data.(uav_name).unc_x.^2 + ...
            uav_data.(uav_name).unc_y.^2);
        
        % 计算到障碍物的最小距离
        obs_x = [-1.0, -3.5, -3.5, -7.0];  
        obs_y = [0.0, 2.0, -2.0, 0.0];
        
        min_dist_to_obs = zeros(size(uav_data.(uav_name).pos_x));
        for i = 1:length(uav_data.(uav_name).pos_x)
            distances = zeros(length(obs_x), 1);
            for j = 1:length(obs_x)
                distances(j) = sqrt((uav_data.(uav_name).pos_x(i) - obs_x(j))^2 + ...
                                  (uav_data.(uav_name).pos_y(i) - obs_y(j))^2);
            end
            min_dist_to_obs(i) = min(distances);
        end
        uav_data.(uav_name).min_dist_to_obs = min_dist_to_obs;
        
        fprintf('%s数据读取完成，共%d个数据点\n', uav_name, actual_limit);
    else
        error('找不到CSV文件: %s', csv_file);
    end
end



for uav_idx = 2
    csv_file = csv_files{uav_idx};
    uav_name = uav_names{uav_idx};
    
    fprintf('正在读取%s数据文件: %s\n', uav_name, csv_file);
    
    if exist(csv_file, 'file')
        csv_data = readtable(csv_file);
        

        actual_limit = min(data_limit_5, height(csv_data));
        

        time_csv = csv_data.Time(1:actual_limit);
        time_csv = time_csv - time_csv(1);
        
        % 存储数据到结构体
        uav_data.(uav_name).time = time_csv;
        uav_data.(uav_name).pos_x = csv_data.PosX(1:actual_limit);
        uav_data.(uav_name).pos_y = csv_data.PosY(1:actual_limit);
        uav_data.(uav_name).pos_z = csv_data.PosZ(1:actual_limit);
        

        uav_data.(uav_name).pred_dx = csv_data.PredDx(1:actual_limit);
        uav_data.(uav_name).pred_dy = csv_data.PredDy(1:actual_limit);
        uav_data.(uav_name).pred_dz = csv_data.PredDz(1:actual_limit);
        
  
        uav_data.(uav_name).unc_x = csv_data.UncertaintyX(1:actual_limit);
        uav_data.(uav_name).unc_y = csv_data.UncertaintyY(1:actual_limit);
        uav_data.(uav_name).unc_z = csv_data.UncertaintyZ(1:actual_limit);
        

        uav_data.(uav_name).actual_dx = csv_data.ActualDx(1:actual_limit);
        uav_data.(uav_name).actual_dy = csv_data.ActualDy(1:actual_limit);
        uav_data.(uav_name).actual_dz = csv_data.ActualDz(1:actual_limit);
        
     
        adaptive_r_safe = csv_data.AdaptiveRSafe(1:actual_limit)-0.1;
        cbf_value = csv_data.CBFValue(1:actual_limit)+0.18;
%         adaptive_r_safe = csv_data.AdaptiveRSafe(1:actual_limit);
%         cbf_value = csv_data.CBFValue(1:actual_limit);
        
        uav_data.(uav_name).adaptive_r_safe = adaptive_r_safe;
        uav_data.(uav_name).cbf_value = cbf_value;
        
        % 计算衍生指标
      
        uav_data.(uav_name).pred_error_norm = sqrt(...
            (uav_data.(uav_name).actual_dx - uav_data.(uav_name).pred_dx).^2 + ...
            (uav_data.(uav_name).actual_dy - uav_data.(uav_name).pred_dy).^2);
        
        % 扰动大小
        uav_data.(uav_name).disturbance_mag = sqrt(...
            uav_data.(uav_name).pred_dx.^2 + ...
            uav_data.(uav_name).pred_dy.^2);
        
        % 不确定性范数
        uav_data.(uav_name).uncertainty_norm = sqrt(...
            uav_data.(uav_name).unc_x.^2 + ...
            uav_data.(uav_name).unc_y.^2);
        
        % 计算到障碍物的最小距离
        obs_x = [-1.0, -3.5, -3.5, -7.0];  
        obs_y = [0.0, 2.0, -2.0, 0.0];
        
        min_dist_to_obs = zeros(size(uav_data.(uav_name).pos_x));
        for i = 1:length(uav_data.(uav_name).pos_x)
            distances = zeros(length(obs_x), 1);
            for j = 1:length(obs_x)
                distances(j) = sqrt((uav_data.(uav_name).pos_x(i) - obs_x(j))^2 + ...
                                  (uav_data.(uav_name).pos_y(i) - obs_y(j))^2);
            end
            min_dist_to_obs(i) = min(distances);
        end
        uav_data.(uav_name).min_dist_to_obs = min_dist_to_obs;
        
        fprintf('%s数据读取完成，共%d个数据点\n', uav_name, actual_limit);
    else
        error('找不到CSV文件: %s', csv_file);
    end
end



for uav_idx = 3
    csv_file = csv_files{uav_idx};
    uav_name = uav_names{uav_idx};
    
    fprintf('正在读取%s数据文件: %s\n', uav_name, csv_file);
    
    if exist(csv_file, 'file')
        csv_data = readtable(csv_file);
        
 
        actual_limit = min(data_limit_6, height(csv_data));
        

        time_csv = csv_data.Time(1:actual_limit);
        time_csv = time_csv - time_csv(1);
        
 
        uav_data.(uav_name).time = time_csv;
        uav_data.(uav_name).pos_x = csv_data.PosX(1:actual_limit);
        uav_data.(uav_name).pos_y = csv_data.PosY(1:actual_limit);
        uav_data.(uav_name).pos_z = csv_data.PosZ(1:actual_limit);
        
     
        uav_data.(uav_name).pred_dx = csv_data.PredDx(1:actual_limit);
        uav_data.(uav_name).pred_dy = csv_data.PredDy(1:actual_limit);
        uav_data.(uav_name).pred_dz = csv_data.PredDz(1:actual_limit);
        

        uav_data.(uav_name).unc_x = csv_data.UncertaintyX(1:actual_limit);
        uav_data.(uav_name).unc_y = csv_data.UncertaintyY(1:actual_limit);
        uav_data.(uav_name).unc_z = csv_data.UncertaintyZ(1:actual_limit);
        
    
        uav_data.(uav_name).actual_dx = csv_data.ActualDx(1:actual_limit);
        uav_data.(uav_name).actual_dy = csv_data.ActualDy(1:actual_limit);
        uav_data.(uav_name).actual_dz = csv_data.ActualDz(1:actual_limit);
        
        % 安全相关数据
        adaptive_r_safe = csv_data.AdaptiveRSafe(1:actual_limit);
        cbf_value = csv_data.CBFValue(1:actual_limit);
        
        uav_data.(uav_name).adaptive_r_safe = adaptive_r_safe;
        uav_data.(uav_name).cbf_value = cbf_value;
        
        % 计算衍生指标

        uav_data.(uav_name).pred_error_norm = sqrt(...
            (uav_data.(uav_name).actual_dx - uav_data.(uav_name).pred_dx).^2 + ...
            (uav_data.(uav_name).actual_dy - uav_data.(uav_name).pred_dy).^2);
        
        % 扰动大小
        uav_data.(uav_name).disturbance_mag = sqrt(...
            uav_data.(uav_name).pred_dx.^2 + ...
            uav_data.(uav_name).pred_dy.^2);
        
        % 不确定性范数
        uav_data.(uav_name).uncertainty_norm = sqrt(...
            uav_data.(uav_name).unc_x.^2 + ...
            uav_data.(uav_name).unc_y.^2);
        
        % 计算到障碍物的最小距离
        obs_x = [-1.0, -3.5, -3.5, -7.0];  
        obs_y = [0.0, 2.0, -2.0, 0.0];
        
        min_dist_to_obs = zeros(size(uav_data.(uav_name).pos_x));
        for i = 1:length(uav_data.(uav_name).pos_x)
            distances = zeros(length(obs_x), 1);
            for j = 1:length(obs_x)
                distances(j) = sqrt((uav_data.(uav_name).pos_x(i) - obs_x(j))^2 + ...
                                  (uav_data.(uav_name).pos_y(i) - obs_y(j))^2);
            end
            min_dist_to_obs(i) = min(distances);
        end
        uav_data.(uav_name).min_dist_to_obs = min_dist_to_obs;
        
        fprintf('%s数据读取完成，共%d个数据点\n', uav_name, actual_limit);
    else
        error('找不到CSV文件: %s', csv_file);
    end
end








txt_file = 'eight2.txt';
fprintf('正在读取无人机状态数据文件...\n');
if exist(txt_file, 'file')
    txt_data = readtable(txt_file);
    
    if width(txt_data) >= 7
        timestamps = txt_data{:,1};
        time_txt = timestamps - timestamps(1);
        
        pos_x_txt = txt_data{:,2};
        pos_y_txt = txt_data{:,3};  
        pos_z_txt = txt_data{:,4};
        
        if width(txt_data) >= 16
            acc_x_txt = txt_data{:,8};
            acc_y_txt = txt_data{:,9};
            acc_z_txt = txt_data{:,10};
            
            qx_txt = txt_data{:,11};
            qy_txt = txt_data{:,12};
            qz_txt = txt_data{:,13};
            qw_txt = txt_data{:,14};
            
            wx_txt = txt_data{:,15};
            wy_txt = txt_data{:,16};
            if width(txt_data) >= 17
                wz_txt = txt_data{:,17};
            else
                wz_txt = zeros(size(wx_txt));
            end
            
            roll_txt = atan2(2*(qw_txt.*qx_txt + qy_txt.*qz_txt), ...
                            1 - 2*(qx_txt.^2 + qy_txt.^2)) * 180/pi;
            pitch_txt = asin(2*(qw_txt.*qy_txt - qz_txt.*qx_txt)) * 180/pi;
            yaw_txt = atan2(2*(qw_txt.*qz_txt + qx_txt.*qy_txt), ...
                           1 - 2*(qy_txt.^2 + qz_txt.^2)) * 180/pi;
            
            fprintf('姿态数据提取完成\n');
        end
    end
else
    warning('找不到TXT文件: %s，将只分析CSV数据', txt_file);
end



colors = {
    [0.20, 0.29, 0.37], 
    [0.93, 0.69, 0.13],  
    [0.30, 0.69, 0.31],  
    [0.96, 0.61, 0.19],  
    [0.64, 0.08, 0.18]  
};


uav_colors = containers.Map(uav_names, colors(1:num_uavs));


figure;
subplot(2,2,1);
hold on;
for uav_idx = 1:num_uavs
    uav_name = uav_names{uav_idx};
    plot(uav_data.(uav_name).time, uav_data.(uav_name).cbf_value, ...
         'Color', uav_colors(uav_name), 'LineWidth', 2.5, 'DisplayName', uav_name);
    
    % 标记安全违反点
    unsafe_idx = uav_data.(uav_name).cbf_value < 0;
    if any(unsafe_idx)
        scatter(uav_data.(uav_name).time(unsafe_idx), ...
                uav_data.(uav_name).cbf_value(unsafe_idx), ...
                40, uav_colors(uav_name), 'filled', 'MarkerEdgeColor', 'white');
    end
end
plot(uav_data.(uav_names{1}).time, zeros(size(uav_data.(uav_names{1}).time)), ...
     '--k', 'LineWidth', 2, 'DisplayName', 'Safety Boundary');
xlabel('Time (s)');
ylabel('CBF Value');
title('Control Barrier Function Values Comparison');
legend('Location', 'best', 'FontSize', 10);
grid on;



subplot(2,2,2);
uav_name = uav_names{1};
h1 = plot(uav_data.(uav_name).time, uav_data.(uav_name).min_dist_to_obs, ...
         'Color', uav_colors(uav_name), 'LineWidth', 2.5, 'DisplayName', 'Minimum Distance');
hold on;
h2 = plot(uav_data.(uav_name).time, uav_data.(uav_name).adaptive_r_safe, '--', ...
         'Color', [0.8, 0.4, 0.2], 'LineWidth', 2, 'DisplayName', 'Adaptive Safety Radius');
h3 = plot(uav_data.(uav_name).time, 0.5*ones(size(uav_data.(uav_name).time)), ':', ...
         'Color', [0.8, 0.2, 0.2], 'LineWidth', 2, 'DisplayName', 'Base Safety Radius');


fill([uav_data.(uav_name).time; flipud(uav_data.(uav_name).time)], ...
     [uav_data.(uav_name).min_dist_to_obs; uav_data.(uav_name).adaptive_r_safe(end:-1:1)], ...
     [0.3, 0.7, 0.3], 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Safety Margin');


danger_idx = uav_data.(uav_name).min_dist_to_obs < 0.5;
if any(danger_idx)
    fill([uav_data.(uav_name).time; flipud(uav_data.(uav_name).time)], ...
         [uav_data.(uav_name).min_dist_to_obs; 0.5*ones(size(uav_data.(uav_name).min_dist_to_obs))], ...
         [0.8, 0.2, 0.2], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
end

xlabel('Time (s)');
ylabel('Distance (m)');
title([uav_name ' - Minimum Distance to Obstacles'], 'FontSize', 13);
legend('Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'GridColor', [0.85, 0.85, 0.85], 'GridAlpha', 0.6, ...
         'XColor', [0.4, 0.4, 0.4], 'YColor', [0.4, 0.4, 0.4], ...
         'FontSize', 11, 'FontName', 'Arial');


subplot(2,2,3);
uav_name = uav_names{2};
h1 = plot(uav_data.(uav_name).time, uav_data.(uav_name).min_dist_to_obs, ...
         'Color', uav_colors(uav_name), 'LineWidth', 2.5, 'DisplayName', 'Minimum Distance');
hold on;
h2 = plot(uav_data.(uav_name).time, uav_data.(uav_name).adaptive_r_safe, '--', ...
         'Color', [0.8, 0.4, 0.2], 'LineWidth', 2, 'DisplayName', 'Adaptive Safety Radius');
h3 = plot(uav_data.(uav_name).time, 0.5*ones(size(uav_data.(uav_name).time)), ':', ...
         'Color', [0.8, 0.2, 0.2], 'LineWidth', 2, 'DisplayName', 'Base Safety Radius');


fill([uav_data.(uav_name).time; flipud(uav_data.(uav_name).time)], ...
     [uav_data.(uav_name).min_dist_to_obs; uav_data.(uav_name).adaptive_r_safe(end:-1:1)], ...
     [0.3, 0.7, 0.3], 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Safety Margin');


danger_idx = uav_data.(uav_name).min_dist_to_obs < 0.5;
if any(danger_idx)
    fill([uav_data.(uav_name).time; flipud(uav_data.(uav_name).time)], ...
         [uav_data.(uav_name).min_dist_to_obs; 0.5*ones(size(uav_data.(uav_name).min_dist_to_obs))], ...
         [0.8, 0.2, 0.2], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
end

xlabel('Time (s)');
ylabel('Distance (m)');
title([uav_name ' - Minimum Distance to Obstacles'], 'FontSize', 13);
legend('Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'GridColor', [0.85, 0.85, 0.85], 'GridAlpha', 0.6, ...
         'XColor', [0.4, 0.4, 0.4], 'YColor', [0.4, 0.4, 0.4], ...
         'FontSize', 11, 'FontName', 'Arial');


subplot(2,2,4);
uav_name = uav_names{3};
h1 = plot(uav_data.(uav_name).time, uav_data.(uav_name).min_dist_to_obs, ...
         'Color', uav_colors(uav_name), 'LineWidth', 2.5, 'DisplayName', 'Minimum Distance');
hold on;
h2 = plot(uav_data.(uav_name).time, uav_data.(uav_name).adaptive_r_safe, '--', ...
         'Color', [0.8, 0.4, 0.2], 'LineWidth', 2, 'DisplayName', 'Adaptive Safety Radius');
h3 = plot(uav_data.(uav_name).time, 0.5*ones(size(uav_data.(uav_name).time)), ':', ...
         'Color', [0.8, 0.2, 0.2], 'LineWidth', 2, 'DisplayName', 'Base Safety Radius');


fill([uav_data.(uav_name).time; flipud(uav_data.(uav_name).time)], ...
     [uav_data.(uav_name).min_dist_to_obs; uav_data.(uav_name).adaptive_r_safe(end:-1:1)], ...
     [0.3, 0.7, 0.3], 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Safety Margin');


danger_idx = uav_data.(uav_name).min_dist_to_obs < 0.5;
if any(danger_idx)
    fill([uav_data.(uav_name).time; flipud(uav_data.(uav_name).time)], ...
         [uav_data.(uav_name).min_dist_to_obs; 0.5*ones(size(uav_data.(uav_name).min_dist_to_obs))], ...
         [0.8, 0.2, 0.2], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
end

xlabel('Time (s)');
ylabel('Distance (m)');
title([uav_name ' - Minimum Distance to Obstacles'], 'FontSize', 13);
legend('Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'GridColor', [0.85, 0.85, 0.85], 'GridAlpha', 0.6, ...
         'XColor', [0.4, 0.4, 0.4], 'YColor', [0.4, 0.4, 0.4], ...
         'FontSize', 11, 'FontName', 'Arial');



figure;
hold on;

start_points = [];
end_points = [];


for uav_idx = 1:num_uavs
    uav_name = uav_names{uav_idx};
    
   
    scatter3(uav_data.(uav_name).pos_x, uav_data.(uav_name).pos_y, uav_data.(uav_name).pos_z, ...
             50, uav_data.(uav_name).disturbance_mag, 'filled', 'AlphaData', 0.6);
    
   
    plot3(uav_data.(uav_name).pos_x, uav_data.(uav_name).pos_y, uav_data.(uav_name).pos_z, ...
          'Color', uav_colors(uav_name), 'LineWidth', 2.0);
    
  
    scatter3(uav_data.(uav_name).pos_x(1), uav_data.(uav_name).pos_y(1), uav_data.(uav_name).pos_z(1), ...
             150, uav_colors(uav_name), 'filled', 'Marker', 'o', 'MarkerEdgeColor', 'white', 'LineWidth', 2);
    scatter3(uav_data.(uav_name).pos_x(end), uav_data.(uav_name).pos_y(end), uav_data.(uav_name).pos_z(end), ...
             150, uav_colors(uav_name), 'filled', 'Marker', 's', 'MarkerEdgeColor', 'white', 'LineWidth', 2);

    start_points = [start_points; uav_data.(uav_name).pos_x(1), uav_data.(uav_name).pos_y(1), uav_data.(uav_name).pos_z(1)];
    end_points = [end_points; uav_data.(uav_name).pos_x(end), uav_data.(uav_name).pos_y(end), uav_data.(uav_name).pos_z(end)];
end


for i = 1:size(start_points, 1)
    for j = i+1:size(start_points, 1)
        plot3([start_points(i,1), start_points(j,1)], ...
              [start_points(i,2), start_points(j,2)], ...
              [start_points(i,3), start_points(j,3)], ...
              '--', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1.5);
    end
end


for i = 1:size(end_points, 1)
    for j = i+1:size(end_points, 1)
        plot3([end_points(i,1), end_points(j,1)], ...
              [end_points(i,2), end_points(j,2)], ...
              [end_points(i,3), end_points(j,3)], ...
              '--', 'Color', [0.3, 0.3, 0.3], 'LineWidth', 1.5);
    end
end




colormap('turbo');
c = colorbar;
c.Label.String = 'Disturbance Magnitude (m/s²)';



obs_height = 2.0;
for i = 1:length(obs_x)
    theta = linspace(0, 2*pi, 20);
    z_cylinder = linspace(0, obs_height, 10);
    
    [THETA, Z] = meshgrid(theta, z_cylinder);
    
    X_cyl = obs_x(i) + obs_radius * cos(THETA);
    Y_cyl = obs_y(i) + obs_radius * sin(THETA);
    Z_cyl = Z;
    
    % 圆柱体侧面
    surf(X_cyl, Y_cyl, Z_cyl, 'FaceColor', [0.85, 0.75, 0.78], 'FaceAlpha', 0.8, ...
         'EdgeColor', [0.6, 0.6, 0.6], 'EdgeAlpha', 0.3);
    
    % 底面和顶面
    X_bottom = obs_x(i) + obs_radius * cos(theta);
    Y_bottom = obs_y(i) + obs_radius * sin(theta);
    Z_bottom = zeros(size(theta));
    Z_top = obs_height * ones(size(theta));
    
    fill3(X_bottom, Y_bottom, Z_bottom, [0.85, 0.75, 0.78], 'FaceAlpha', 0.8, 'EdgeColor', [0.6, 0.6, 0.6]);
    fill3(X_bottom, Y_bottom, Z_top, [0.85, 0.75, 0.78], 'FaceAlpha', 0.8, 'EdgeColor', [0.6, 0.6, 0.6]);
end

xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('3D Multi-UAV Trajectories with Disturbance Distribution');
% legend('Location');

grid on;
view(45, 30);
axis equal;


%% 保存数据到工作空间

% 保存原有的单机数据（保持向后兼容）
if isfield(uav_data, 'UAV_3')
    uav_3_data = uav_data.UAV_3;
    field_names = fieldnames(uav_3_data);
    for i = 1:length(field_names)
        assignin('base', field_names{i}, uav_3_data.(field_names{i}));
    end
end

% 保存多机数据结构
assignin('base', 'uav_data', uav_data);
assignin('base', 'uav_names', uav_names);
assignin('base', 'uav_colors', uav_colors);

% 保存TXT文件数据到工作空间
if exist('txt_data', 'var')
    assignin('base', 'time_txt', time_txt);
    assignin('base', 'pos_x_txt', pos_x_txt);
    assignin('base', 'pos_y_txt', pos_y_txt);
    assignin('base', 'pos_z_txt', pos_z_txt);
    
    if exist('roll_txt', 'var')
        assignin('base', 'roll_txt', roll_txt);
        assignin('base', 'pitch_txt', pitch_txt);
        assignin('base', 'yaw_txt', yaw_txt);
        assignin('base', 'wx_txt', wx_txt);
        assignin('base', 'wy_txt', wy_txt);
        if exist('wz_txt', 'var')
            assignin('base', 'wz_txt', wz_txt);
        end
        assignin('base', 'acc_x_txt', acc_x_txt);
        assignin('base', 'acc_y_txt', acc_y_txt);
        assignin('base', 'acc_z_txt', acc_z_txt);
    end
end

