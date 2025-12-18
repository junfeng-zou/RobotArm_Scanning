function execute_scanning_trajectory(trajectory_file, speed_ratio)
% EXECUTE_SCANNING_TRAJECTORY 执行保存的扫描轨迹
%
% 功能: 读取轨迹文件并控制机械臂执行扫描运动
%
% 输入:
%   trajectory_file - 轨迹文件路径 (例如: 'scanning_trajectory_20251217_200823.mat')
%   speed_ratio - (可选) 速度比例 1-100，默认为30
%
% 用法示例:
%   execute_scanning_trajectory('scanning_trajectory_20251217_200823.mat')
%   execute_scanning_trajectory('scanning_trajectory_20251217_200823.mat', 50)
%
% 注意: 执行前请确保机器人处于安全位置！

%% ====== 参数检查 ======

if nargin < 1
    error('请提供轨迹文件路径！');
end

if nargin < 2
    speed_ratio = 30;  % 默认速度比例30%
end

%% ====== 加载轨迹数据 ======

fprintf('====== 加载轨迹数据 ======\n');

if ~exist(trajectory_file, 'file')
    error('找不到轨迹文件: %s', trajectory_file);
end

% 加载MAT文件
data = load(trajectory_file);

if ~isfield(data, 'joint_trajectory')
    error('轨迹文件中缺少joint_trajectory字段！');
end

joint_trajectory = data.joint_trajectory;
num_points = size(joint_trajectory, 1);

fprintf('轨迹文件: %s\n', trajectory_file);
fprintf('轨迹点数: %d\n', num_points);

% 如果存在工作空间轨迹，也加载
if isfield(data, 'workspace_trajectory')
    workspace_trajectory = data.workspace_trajectory;
    fprintf('工作空间轨迹已加载\n');
else
    workspace_trajectory = [];
end

fprintf('\n');

%% ====== 连接机械臂 ======

fprintf('====== 连接机械臂 ======\n');

% 创建机器人对象
robot = ZJFDobotCR5();

% 连接到机器人
try
    robot.Connect();
    fprintf('机器人连接成功！\n');
catch ME
    error('连接机器人失败: %s', ME.message);
end

% 等待连接稳定
pause(1.0);

%% ====== 初始化机器人 ======

fprintf('\n====== 初始化机器人 ======\n');

% 清除错误
robot.ClearError();
pause(0.5);

% 使能机器人 (负载设为5kg，根据实际情况调整)
robot.Enable(0.05);
pause(1.0);

% 设置速度比例
robot.SetSpeedRatio(speed_ratio);
fprintf('速度比例设置为: %d%%\n', speed_ratio);
pause(0.5);

% 检查机器人状态
fprintf('当前机器人状态: %s\n', robot.RobotMode);

if ~strcmp(robot.RobotMode, 'ENABLE') && ~strcmp(robot.RobotMode, 'RUNNING')
    warning('机器人状态异常: %s', robot.RobotMode);
    response = input('是否继续执行? (y/n): ', 's');
    if ~strcmpi(response, 'y')
        robot.Disconnect();
        error('用户取消执行');
    end
end

fprintf('\n');

%% ====== 执行轨迹 ======

fprintf('====== 开始执行扫描轨迹 (ServoJ模式) ======\n');
% ServoJ 参数配置（固定值）
servo_t = 0.01;            % ServoJ指令周期 (s) - 每个轨迹点发布间隔
servo_lookahead = 0.1;     % 前瞻时间 (s) - 轨迹平滑参数
servo_gain = 300;          % 增益 - 控制响应速度

fprintf('ServoJ参数: t=%.3fs, lookahead=%.2fs, gain=%d\n\n', ...
    servo_t, servo_lookahead, servo_gain);

% 记录起始时间
start_time = tic;

% 创建实时图表（如果有工作空间轨迹）
if ~isempty(workspace_trajectory)
    figure('Name', '实时轨迹执行', 'NumberTitle', 'off');
    
    % 绘制计划轨迹
    plot3(workspace_trajectory(:, 1), workspace_trajectory(:, 2), workspace_trajectory(:, 3), ...
        'b--', 'LineWidth', 1.5);
    hold on;
    
    % 实时位置点
    h_current = plot3(0, 0, 0, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    % 已执行轨迹
    h_executed = plot3(0, 0, 0, 'g-', 'LineWidth', 2);
    
    grid on;
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    title('扫描轨迹执行进度 (ServoJ模式)');
    legend('计划轨迹', '当前位置', '已执行轨迹');
    axis equal;
    view(45, 30);
    drawnow;
    
    executed_positions = [];
end

% 先移动到起始位置
fprintf('移动到起始位置...\n');
start_joints_deg = rad2deg(joint_trajectory(1, :));
robot.JointMovJ(start_joints_deg);
pause(3.0);  % 等待到达起始位置
fprintf('到达起始位置，开始ServoJ控制\n\n');

% 使用ServoJ逐点执行轨迹，每个点间隔 servo_t
for i = 1:num_points
    % 记录本次循环开始时间
    loop_start = tic;
    
    % 获取目标关节角度
    target_joints = joint_trajectory(i, :);
    target_joints_deg = rad2deg(target_joints);
    
    try
        % 使用ServoJ发送运动指令
        robot.ServoJ(target_joints_deg, servo_t, servo_lookahead, servo_gain);
        
        % 显示进度（每10个点显示一次，避免输出过多）
        if mod(i, 10) == 0 || i == 1 || i == num_points
            % 获取当前实际位置
            current_joint_angles = robot.JointAngles;
            current_pose = robot.CartesianPose;
            
            fprintf('点 %d/%d (%.1f%%) - 关节: [%s] deg\n', ...
                i, num_points, i/num_points*100, ...
                sprintf('%.1f ', current_joint_angles));
            
            % 更新实时图表
            if ~isempty(workspace_trajectory)
                % 添加到已执行轨迹
                executed_positions = [executed_positions; current_pose(1:3)];
                
                % 更新图表
                set(h_current, 'XData', current_pose(1), 'YData', current_pose(2), 'ZData', current_pose(3));
                set(h_executed, 'XData', executed_positions(:, 1), ...
                    'YData', executed_positions(:, 2), 'ZData', executed_positions(:, 3));
                drawnow;
            end
        end
        
        % 检查机器人状态
        if strcmp(robot.RobotMode, 'ERROR')
            error('机器人出现错误，停止执行！');
        end
        
    catch ME
        fprintf('\n执行出错: %s\n', ME.message);
        robot.StopMove();
        robot.Disconnect();
        error('轨迹执行失败！');
    end
    
    % 等待直到本次循环时间达到 servo_t
    elapsed = toc(loop_start);
    remaining = servo_t - elapsed;
    if remaining > 0
        pause(remaining);
    end
end

% 停止ServoJ模式
fprintf('\n停止ServoJ控制...\n');
robot.StopMove();
pause(0.5);

% 记录总时间
elapsed_time = toc(start_time);

fprintf('\n====== 轨迹执行完成！ ======\n');
fprintf('总执行时间: %.2f 秒\n', elapsed_time);
fprintf('轨迹点数: %d\n', num_points);
fprintf('平均每点用时: %.3f 秒\n', elapsed_time / num_points);

%% ====== 返回原点（可选） ======

response = input('\n是否返回零位？(y/n): ', 's');
if strcmpi(response, 'y')
    fprintf('返回零位中...\n');
    robot.JointMovJ([0, 0, 0, 0, 0, 0]);
    pause(3.0);
end

%% ====== 断开连接 ======

fprintf('\n====== 断开连接 ======\n');
robot.Disable();
pause(0.5);
robot.Disconnect();
fprintf('机器人已断开连接\n');

fprintf('\n====== 扫描任务完成！ ======\n');

end
