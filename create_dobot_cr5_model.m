function robot = create_dobot_cr5_model()
% CREATE_DOBOT_CR5_MODEL 创建Dobot CR5机械臂的Robotics Toolbox模型
%
% 输出:
%   robot - SerialLink机器人模型对象
%
% 使用示例:
%   robot = create_dobot_cr5_model();
%   robot.plot([0 0 0 0 0 0]);  % 显示机器人
%
% DH参数表 (Modified DH):
%     d(m)    a(m)    alpha(rad)  theta
%     0.147   0       0           theta1
%     0       0       pi/2        pi/2+theta2
%     0.025   0.427   0           theta3
%     0.116   0.357   0           pi/2+theta4
%     0.116   0       pi/2        theta5
%     0.105   0       -pi/2       theta6

% 定义DH参数 (Modified DH约定)
% 参数: theta, d, a, alpha
L(1) = Link('d', 0.147, 'a', 0, 'alpha', 0, 'offset', 0,'modified');
L(2) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2,'modified');
L(3) = Link('d', 0.025, 'a', 0.427, 'alpha', 0, 'offset', 0,'modified');
L(4) = Link('d', 0.116, 'a', 0.357, 'alpha', 0, 'offset', pi/2,'modified');
L(5) = Link('d', 0.116, 'a', 0, 'alpha', pi/2, 'offset', 0,'modified');
L(6) = Link('d', 0.105, 'a', 0, 'alpha', -pi/2, 'offset', 0,'modified');

% 设置关节限位 (单位: 弧度)
L(1).qlim = deg2rad([-180, 180]);
L(2).qlim = deg2rad([-180, 180]);
L(3).qlim = deg2rad([-180, 180]);
L(4).qlim = deg2rad([-180, 180]);
L(5).qlim = deg2rad([-180, 180]);
L(6).qlim = deg2rad([-360, 360]);

% 创建SerialLink机器人对象
robot = SerialLink(L, 'name', 'Dobot CR5');

% 设置基座变换
robot.base = eye(4);

% 设置工具变换（如果相机有偏移，可以在这里设置）
robot.tool = eye(4);

end
