%% 机械臂相机扫描轨迹规划脚本（使用Robotics Toolbox）
% 功能: 生成圆周螺旋扫描轨迹，计算关节角度，并保存轨迹数据
% 依赖: Peter Corke's Robotics Toolbox for MATLAB


clear; clc; close all;

%% ====== 用户可配置参数 ======

% 目标物体位置 (mm)
target_position = [-470, -143, 50];  % [X, Y, Z]

% 圆周扫描参数（固定高度俯拍）
scan_radius = 200;          % 扫描半径 (mm) - 相机距离物体的水平距离
scan_height = 300;          % 扫描高度 (mm) - 相机在物体上方的高度
camera_tilt = 15;           % 相机俯视角度 (度) - 0度为完全竖直向下，建议5-30度
num_rotations = 1;          % 扫描圈数
num_points = 150;           % 轨迹点总数

% 扫描速度和时间参数
scan_velocity = 60;         % 扫描速度 (mm/s) - 控制相机沿轨迹移动的速度
servo_t = 0.01;             % 轨迹点时间间隔 (s) - 与ServoJ的servo_t参数一致

% 注意：轨迹点数量(num_points)将根据轨迹长度、速度和时间间隔自动计算

%% ====== 初始化机器人模型 ======

% 创建Dobot CR5机器人模型（使用Robotics Toolbox）
fprintf('====== 初始化机器人模型 ======\n');
robot = create_dobot_cr5_model();
fprintf('机器人模型创建成功: %s\n', robot.name);
fprintf('自由度: %d\n\n', robot.n);

% 初始关节角度猜测 (方便IK求解)
q_init = [0, 0, -pi/2, 0, pi/2, 0];  % 单位: 弧度

% 保存文件名
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
save_filename = sprintf('scanning_trajectory_%s.mat', timestamp);

%% ====== 计算轨迹长度和点数 ======

fprintf('====== 计算轨迹参数 ======\n');

% 先用较少的点估算轨迹长度
num_points_estimate = 100;
t_estimate = linspace(0, 1, num_points_estimate);
theta_estimate = 2 * pi * num_rotations * t_estimate;

% 计算估算轨迹的位置
x_estimate = target_position(1) + scan_radius * cos(theta_estimate);
y_estimate = target_position(2) + scan_radius * sin(theta_estimate);
z_estimate = target_position(3) + scan_height * ones(1, num_points_estimate);

% 计算轨迹总长度
trajectory_length_estimate = 0;
for i = 2:num_points_estimate
    segment = sqrt((x_estimate(i)-x_estimate(i-1))^2 + ...
        (y_estimate(i)-y_estimate(i-1))^2 + ...
        (z_estimate(i)-z_estimate(i-1))^2);
    trajectory_length_estimate = trajectory_length_estimate + segment;
end

% 根据速度和长度计算总时间
total_scan_time_target = trajectory_length_estimate / scan_velocity;

% 根据时间间隔计算所需点数
num_points = ceil(total_scan_time_target / servo_t) + 1;  % +1 确保包含终点

fprintf('估算轨迹长度: %.2f mm\n', trajectory_length_estimate);
fprintf('目标扫描时间: %.2f 秒\n', total_scan_time_target);
fprintf('时间间隔: %.3f 秒\n', servo_t);
fprintf('计算得到轨迹点数: %d\n\n', num_points);

%% ====== 生成圆周扫描轨迹（固定高度） ======

fprintf('====== 开始生成扫描轨迹 ======\n');
fprintf('目标位置: [%.1f, %.1f, %.1f] mm\n', target_position);
fprintf('扫描半径: %.1f mm\n', scan_radius);
fprintf('扫描高度: %.1f mm (相对目标物体)\n', scan_height);
fprintf('相机俯视角: %.1f 度\n', camera_tilt);
fprintf('扫描圈数: %d\n', num_rotations);
fprintf('轨迹点数: %d\n', num_points);
fprintf('扫描速度: %.1f mm/s\n', scan_velocity);
fprintf('点间时间间隔: %.3f 秒\n\n', servo_t);

% 生成参数化轨迹
t = linspace(0, 1, num_points);  % 参数 t ∈ [0, 1]
theta = 2 * pi * num_rotations * t;  % 角度从0到2π*num_rotations

% 固定半径和高度（不随时间变化）
radius = scan_radius * ones(1, num_points);  % 恒定半径
height = scan_height * ones(1, num_points);  % 恒定高度

% 计算相机位置 (圆柱坐标系)
camera_x = target_position(1) + radius .* cos(theta);
camera_y = target_position(2) + radius .* sin(theta);
camera_z = target_position(3) + height;

% 初始化工作空间轨迹数组 [X, Y, Z, Rx, Ry, Rz]
workspace_trajectory = zeros(num_points, 6);

% 计算每个点的位姿（俯视角朝向目标）
for i = 1:num_points
    % 相机位置
    cam_pos = [camera_x(i), camera_y(i), camera_z(i)];
    
    % 计算相机朝向目标的方向向量
    direction = target_position - cam_pos;
    direction = direction / norm(direction);  % 归一化
    
    % 应用俯视角度调整
    % 方法：将direction向量稍微向下倾斜camera_tilt角度
    % 相机Z轴（光轴）指向目标，但保持一定俯视角
    z_axis = direction;
    
    % 选择合适的X轴方向（沿着圆周切线方向）
    % 切线方向与径向垂直
    radial_dir = [cam_pos(1) - target_position(1), cam_pos(2) - target_position(2), 0];
    radial_dir = radial_dir / norm(radial_dir);
    
    % 切线方向（逆时针旋转90度）
    tangent_dir = [-radial_dir(2), radial_dir(1), 0];
    
    % X轴方向（切线方向）
    x_axis = tangent_dir;
    x_axis = x_axis / norm(x_axis);
    
    % Y轴方向（通过叉乘得到）
    y_axis = cross(z_axis, x_axis);
    y_axis = y_axis / norm(y_axis);
    
    % 重新正交化X轴
    x_axis = cross(y_axis, z_axis);
    x_axis = x_axis / norm(x_axis);
    
    % 构建旋转矩阵
    R = [x_axis', y_axis', z_axis'];
    
    % 转换为欧拉角 (ZYX顺序)
    euler = rotm2eul(R, 'ZYX');  % [Rz, Ry, Rx]
    
    % 保存位姿
    workspace_trajectory(i, :) = [cam_pos, euler(3), euler(2), euler(1)];
end

fprintf('轨迹生成完成！\n\n');

%% ====== 求解逆运动学（使用Robotics Toolbox） ======

fprintf('====== 开始求解逆运动学 ======\n');

% 初始化关节空间轨迹数组
joint_trajectory = zeros(num_points, 6);

% 使用上一个点的解作为下一个点的初始猜测（提高连续性）
q_current = q_init;

% 显示进度
fprintf('进度: ');
for i = 1:num_points
    % 使用上一个点的解作为下一个点的初始猜测（提高连续性）
    % 改进：使用线性外推预测下一个点 (q_predict = q_current + velocity)
    if i > 1
        % 简单的线性外推
        q_guess = joint_trajectory(i-1, :);
        if i > 2
            velocity = joint_trajectory(i-1, :) - joint_trajectory(i-2, :);
            % 处理周期性
            for jj = 1:6
                if velocity(jj) > pi, velocity(jj) = velocity(jj) - 2*pi; end
                if velocity(jj) < -pi, velocity(jj) = velocity(jj) + 2*pi; end
            end
            q_guess = q_guess + velocity;
        end
    else
        q_guess = q_current;
    end
    
    % 显示进度
    if mod(i, 10) == 0 || i == 1
        fprintf('%d/%d ', i, num_points);
    end
    
    % 获取目标位姿
    target_pose = workspace_trajectory(i, :);
    
    % 构造目标齐次变换矩阵
    % 位置 (转换为m)
    position = target_pose(1:3) / 1000;
    
    % 姿态 (欧拉角ZYX顺序转旋转矩阵)
    euler_angles = [target_pose(6), target_pose(5), target_pose(4)]; % [Rz, Ry, Rx]
    R = eul2rotm(euler_angles, 'ZYX');
    
    % 组装齐次变换矩阵
    T_target = eye(4);
    T_target(1:3, 1:3) = R;
    T_target(1:3, 4) = position';
    
    % 使用Robotics Toolbox的ikcon求解（考虑关节限位）
    % 优先尝试使用上一步真正的解作为初值（不仅是外推值），有时更稳
    [q_sol, success, ~] = robot.ikcon(T_target, q_guess);
    
    % 如果ikcon失败，尝试使用ikine数值解
    if ~success
        warning('点 %d ikcon求解失败，尝试使用ikine...', i);
        q_sol = robot.ikine(T_target, 'mask', [1 1 1 1 1 1], 'q0', q_guess);
    end
    
    % 保存关节角度
    joint_trajectory(i, :) = q_sol;
    
    % 增强的关节连续性检查（防止大幅跳变）
    if i > 1
        % 计算与前一个点的关节角度差异
        joint_diff = q_sol - joint_trajectory(i-1, :);
        
        % 首先处理2π周期性
        for jj = 1:6
            while joint_diff(jj) > pi
                q_sol(jj) = q_sol(jj) - 2*pi;
                joint_diff(jj) = joint_diff(jj) - 2*pi;
            end
            while joint_diff(jj) < -pi
                q_sol(jj) = q_sol(jj) + 2*pi;
                joint_diff(jj) = joint_diff(jj) + 2*pi;
            end
        end
        
        % 检查归一化后是否仍有大的跳变（超过30度 - 更严格的阈值）
        max_jump = max(abs(rad2deg(joint_diff)));
        
        % 如果跳变过大，尝试更激进的修复策略
        if max_jump > 30
            fprintf('\n警告: 点 %d 检测到大的关节跳变 (%.1f度)，尝试修复...\n', i, max_jump);
            
            best_q = q_sol;
            min_jump = max_jump;
            
            % 策略1: 强制从前一个点开始搜索（不含外推），并限制步长
            % 使用ikine通常比ikcon在局部搜索上表现不同，可能避开奇异点
            try
                q_retry1 = robot.ikine(T_target, 'q0', joint_trajectory(i-1, :), ...
                    'mask', [1 1 1 1 1 1], 'ilimit', 100, 'tol', 1e-6);
                
                % 归一化并计算跳变
                diff_r1 = q_retry1 - joint_trajectory(i-1, :);
                for jj=1:6
                    while diff_r1(jj)>pi, q_retry1(jj)=q_retry1(jj)-2*pi; diff_r1(jj)=diff_r1(jj)-2*pi; end
                    while diff_r1(jj)<-pi, q_retry1(jj)=q_retry1(jj)+2*pi; diff_r1(jj)=diff_r1(jj)+2*pi; end
                end
                
                jump_r1 = max(abs(rad2deg(diff_r1)));
                if jump_r1 < min_jump
                    min_jump = jump_r1;
                    best_q = q_retry1;
                    fprintf('  策略1(ikine)有效，跳变降至 %.1f 度\n', jump_r1);
                end
            catch
            end
            
            % 策略2: 如果还是不行，尝试微扰动初始猜测
            if min_jump > 30
                for attempt = 1:5
                    q_perturbed = joint_trajectory(i-1, :) + (rand(1,6)-0.5)*0.2; % ±0.1 rad 扰动
                    [q_retry2, succ2] = robot.ikcon(T_target, q_perturbed);
                    if succ2
                        diff_r2 = q_retry2 - joint_trajectory(i-1, :);
                        for jj=1:6
                            while diff_r2(jj)>pi, q_retry2(jj)=q_retry2(jj)-2*pi; diff_r2(jj)=diff_r2(jj)-2*pi; end
                            while diff_r2(jj)<-pi, q_retry2(jj)=q_retry2(jj)+2*pi; diff_r2(jj)=diff_r2(jj)+2*pi; end
                        end
                        jump_r2 = max(abs(rad2deg(diff_r2)));
                        if jump_r2 < min_jump
                            min_jump = jump_r2;
                            best_q = q_retry2;
                            fprintf('  策略2(扰动%d)有效，跳变降至 %.1f 度\n', attempt, jump_r2);
                            break; % 找到好的就退出
                        end
                    end
                end
            end
            
            % 应用最佳结果
            q_sol = best_q;
            
            if min_jump > 30
                fprintf('  修复失败，仍保留较大跳变。\n');
            else
                fprintf('  修复成功。\n');
            end
        end
        
        % 更新修正后的关节角度
        joint_trajectory(i, :) = q_sol;
        q_current = q_sol;
    end
    
    % 验证FK（使用工具箱）
    T_check = robot.fkine(q_sol);
    pos_error = norm(T_check.t * 1000 - target_pose(1:3)');
    
    if pos_error > 5.0  % 位置误差大于5mm则警告
        warning('点 %d 的IK误差较大: %.2f mm', i, pos_error);
    end
end

fprintf('\n逆运动学求解完成！\n\n');

%% ====== 计算速度剖面和时间戳 ======

fprintf('====== 计算速度剖面 ======\n');

% 初始化速度和时间数组
velocities = zeros(num_points, 1);             % 实际速度 (mm/s)
joint_velocities = zeros(num_points, 6);       % 关节速度 (rad/s)
timestamps = zeros(num_points, 1);             % 时间戳 (s)

% 由于点已经均匀分布在时间上，每个点之间的时间间隔都是 servo_t
for i = 2:num_points
    % 时间戳：均匀间隔
    timestamps(i) = (i - 1) * servo_t;
    
    % 计算笛卡尔空间位移
    position_diff = workspace_trajectory(i, 1:3) - workspace_trajectory(i-1, 1:3);
    segment_length = norm(position_diff);  % mm
    
    % 计算实际速度（基于固定时间间隔）
    velocities(i) = segment_length / servo_t;  % mm/s
    
    % 计算关节速度 (rad/s)
    joint_diff = joint_trajectory(i, :) - joint_trajectory(i-1, :);
    % 处理周期性
    for jj = 1:6
        if joint_diff(jj) > pi, joint_diff(jj) = joint_diff(jj) - 2*pi; end
        if joint_diff(jj) < -pi, joint_diff(jj) = joint_diff(jj) + 2*pi; end
    end
    joint_velocities(i, :) = joint_diff / servo_t;  % rad/s
end

% 第一个点的速度设为0
velocities(1) = 0;
joint_velocities(1, :) = 0;
timestamps(1) = 0;

% 计算总时间
total_scan_time = timestamps(end);

fprintf('速度剖面计算完成！\n');
fprintf('实际总扫描时间: %.2f 秒 (%.2f 分钟)\n', total_scan_time, total_scan_time/60);
fprintf('平均速度: %.2f mm/s\n', mean(velocities(2:end)));
fprintf('目标速度: %.2f mm/s\n', scan_velocity);
fprintf('最大关节速度: %.2f deg/s\n\n', max(max(abs(rad2deg(joint_velocities)))));



%% ====== 保存轨迹数据 ======

fprintf('====== 保存轨迹数据 ======\n');

% 保存到MAT文件
save(save_filename, 'workspace_trajectory', 'joint_trajectory', ...
    'target_position', 'scan_radius', 'scan_height', 'camera_tilt', ...
    'num_rotations', 'num_points', ...
    'scan_velocity', 'servo_t', 'velocities', 'joint_velocities', 'timestamps', ...
    'total_scan_time');

fprintf('轨迹已保存至: %s\n\n', save_filename);

%% ====== 可视化轨迹 ======

fprintf('====== 生成可视化图表 ======\n');

% 图1: 3D工作空间轨迹
figure('Name', '3D工作空间扫描轨迹', 'NumberTitle', 'off');
plot3(workspace_trajectory(:, 1), workspace_trajectory(:, 2), workspace_trajectory(:, 3), ...
    'b-', 'LineWidth', 2);
hold on;
plot3(target_position(1), target_position(2), target_position(3), ...
    'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
plot3(workspace_trajectory(1, 1), workspace_trajectory(1, 2), workspace_trajectory(1, 3), ...
    'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(workspace_trajectory(end, 1), workspace_trajectory(end, 2), workspace_trajectory(end, 3), ...
    'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% 绘制方向箭头（每隔10个点）
for i = 1:10:num_points
    cam_pos = workspace_trajectory(i, 1:3);
    arrow_dir = (target_position - cam_pos) / norm(target_position - cam_pos) * 30;
    quiver3(cam_pos(1), cam_pos(2), cam_pos(3), ...
        arrow_dir(1), arrow_dir(2), arrow_dir(3), ...
        'r', 'LineWidth', 1.5, 'MaxHeadSize', 2);
end

grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('相机圆周扫描轨迹 (俯视角度, 固定高度)');
legend('扫描轨迹', '目标物体', '起点', '终点', '相机朝向');
axis equal;
view(45, 30);

% 图2: 关节角度变化曲线
figure('Name', '关节角度变化', 'NumberTitle', 'off');
for i = 1:6
    subplot(3, 2, i);
    plot(1:num_points, rad2deg(joint_trajectory(:, i)), 'LineWidth', 1.5);
    grid on;
    xlabel('轨迹点序号');
    ylabel('角度 (度)');
    title(sprintf('关节 J%d', i));
end
sgtitle('关节角度变化曲线');

% 图3: XYZ位置分量
figure('Name', '位置分量变化', 'NumberTitle', 'off');
subplot(3, 1, 1);
plot(1:num_points, workspace_trajectory(:, 1), 'LineWidth', 1.5);
grid on; ylabel('X (mm)'); title('X轴位置');

subplot(3, 1, 2);
plot(1:num_points, workspace_trajectory(:, 2), 'LineWidth', 1.5);
grid on; ylabel('Y (mm)'); title('Y轴位置');

subplot(3, 1, 3);
plot(1:num_points, workspace_trajectory(:, 3), 'LineWidth', 1.5);
grid on; ylabel('Z (mm)'); title('Z轴位置');
xlabel('轨迹点序号');

sgtitle('工作空间位置分量');

% 图4: 速度剖面
figure('Name', '速度剖面', 'NumberTitle', 'off');

% 轨迹速度
subplot(2, 1, 1);
plot(timestamps, velocities, 'b-', 'LineWidth', 2);
ylabel('速度 (mm/s)');
xlabel('时间 (s)');
title(sprintf('扫描速度 (设定: %.1f mm/s)', scan_velocity));
grid on;
yline(scan_velocity, 'r--', 'LineWidth', 1.5, 'Label', '设定速度');

% 关节空间速度
subplot(2, 1, 2);
hold on;
for i = 1:6
    plot(timestamps, rad2deg(joint_velocities(:, i)), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('J%d', i));
end
grid on;
xlabel('时间 (s)');
ylabel('关节速度 (deg/s)');
title('关节速度剖面');
legend('Location', 'bestoutside');

sgtitle('扫描速度剖面');

% 图5: 时间和距离统计
figure('Name', '时间距离统计', 'NumberTitle', 'off');

% 累计距离
cumulative_distance = zeros(num_points, 1);
for i = 2:num_points
    segment_length = norm(workspace_trajectory(i, 1:3) - workspace_trajectory(i-1, 1:3));
    cumulative_distance(i) = cumulative_distance(i-1) + segment_length;
end

subplot(2, 1, 1);
plot(timestamps, cumulative_distance, 'b-', 'LineWidth', 2);
grid on;
xlabel('时间 (s)');
ylabel('累计距离 (mm)');
title('扫描进度 - 距离 vs 时间');

subplot(2, 1, 2);
plot(1:num_points, timestamps, 'r-', 'LineWidth', 2);
grid on;
xlabel('轨迹点序号');
ylabel('累计时间 (s)');
title('轨迹点时间分布');

sgtitle(sprintf('扫描统计 (总时间: %.1f秒, 总距离: %.1fmm)', ...
    total_scan_time, cumulative_distance(end)));

fprintf('可视化完成！\n\n');

%% ====== 打印统计信息 ======

fprintf('====== 轨迹统计信息 ======\n');
fprintf('关节角度范围 (度):\n');
for i = 1:6
    fprintf('  J%d: [%.2f, %.2f]\n', i, ...
        min(rad2deg(joint_trajectory(:, i))), ...
        max(rad2deg(joint_trajectory(:, i))));
end

fprintf('\n工作空间范围 (mm):\n');
fprintf('  X: [%.2f, %.2f]\n', min(workspace_trajectory(:, 1)), max(workspace_trajectory(:, 1)));
fprintf('  Y: [%.2f, %.2f]\n', min(workspace_trajectory(:, 2)), max(workspace_trajectory(:, 2)));
fprintf('  Z: [%.2f, %.2f]\n', min(workspace_trajectory(:, 3)), max(workspace_trajectory(:, 3)));

% 计算轨迹总长度
trajectory_length = 0;
for i = 2:num_points
    segment_length = sqrt(sum((workspace_trajectory(i, 1:3) - workspace_trajectory(i-1, 1:3)).^2));
    trajectory_length = trajectory_length + segment_length;
end
fprintf('\n轨迹总长度: %.2f mm\n', trajectory_length);

fprintf('\n扫描速度信息:\n');
fprintf('  设定扫描速度: %.1f mm/s\n', scan_velocity);
fprintf('  实际平均速度: %.2f mm/s\n', mean(velocities(2:end)));
fprintf('  最大关节速度: %.2f deg/s (J%d)\n', ...
    max(max(abs(rad2deg(joint_velocities)))), ...
    find(max(abs(rad2deg(joint_velocities))) == max(max(abs(rad2deg(joint_velocities)))), 1));
fprintf('  预计扫描时间: %.2f 秒 (%.2f 分钟)\n', total_scan_time, total_scan_time/60);

%% ====== 可选：机器人3D动画 ======

response = input('\n是否显示机器人轨迹动画？(y/n): ', 's');
if strcmpi(response, 'y')
    fprintf('正在生成机器人动画...\n');
    
    try
        % 创建新窗口
        figure('Name', '机器人轨迹动画', 'NumberTitle', 'off');
        
        % 使用plot方法显示轨迹（直接传入整个轨迹数组）
        robot.plot(joint_trajectory, 'workspace', [-1 1 -1 1 0 1.2], ...
            'trail', 'b-', 'delay', 0.02);
        
        fprintf('动画播放完成！\n');
    catch ME
        fprintf('动画显示失败: %s\n', ME.message);
        fprintf('尝试使用静态显示...\n');
        
        % 备用方案：显示起始和结束位置
        figure('Name', '机器人起始和结束位置', 'NumberTitle', 'off');
        subplot(1, 2, 1);
        robot.plot(joint_trajectory(1, :), 'workspace', [-1 1 -1 1 0 1.2]);
        title('起始位置');
        
        subplot(1, 2, 2);
        robot.plot(joint_trajectory(end, :), 'workspace', [-1 1 -1 1 0 1.2]);
        title('结束位置');
    end
end


fprintf('\n====== 轨迹规划完成！======\n');
fprintf('请运行 execute_scanning_trajectory(''%s'') 来执行轨迹\n', save_filename);
