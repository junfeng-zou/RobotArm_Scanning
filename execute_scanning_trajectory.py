"""
扫描轨迹执行脚本 - Python版本
功能: 读取轨迹文件并控制机械臂执行扫描运动
"""

import numpy as np
import scipy.io as sio
import time
import sys
from dobot_cr5 import DobotCR5

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not available, visualization disabled")


def execute_scanning_trajectory(trajectory_file: str, speed_ratio: int = 30):
    """
    执行保存的扫描轨迹
    
    Args:
        trajectory_file: 轨迹文件路径 (例如: 'scanning_trajectory_20251217_200823.mat')
        speed_ratio: 速度比例 1-100，默认为30
    """
    
    # ====== 参数检查 ======
    print("====== 加载轨迹数据 ======")
    
    # 加载MAT文件
    try:
        data = sio.loadmat(trajectory_file)
    except FileNotFoundError:
        print(f"错误: 找不到轨迹文件 {trajectory_file}")
        return
    except Exception as e:
        print(f"错误: 加载轨迹文件失败 - {e}")
        return
    
    # 检查必需字段
    if 'joint_trajectory' not in data:
        print("错误: 轨迹文件中缺少joint_trajectory字段！")
        return
    
    joint_trajectory = data['joint_trajectory']
    num_points = joint_trajectory.shape[0]
    
    print(f"轨迹文件: {trajectory_file}")
    print(f"轨迹点数: {num_points}")
    
    # 加载工作空间轨迹（如果存在）
    workspace_trajectory = data.get('workspace_trajectory', None)
    if workspace_trajectory is not None:
        print("工作空间轨迹已加载")
    
    print()
    
    # ====== 连接机械臂 ======
    print("====== 连接机械臂 ======")
    
    robot = DobotCR5()
    
    try:
        robot.connect()
        print("机器人连接成功！")
    except Exception as e:
        print(f"连接机器人失败: {e}")
        return
    
    # 等待连接稳定
    time.sleep(1.0)
    
    # ====== 初始化机器人 ======
    print("\n====== 初始化机器人 ======")
    
    # 清除错误
    robot.clear_error()
    time.sleep(0.5)
    
    # 使能机器人 (负载设为0.05kg)
    robot.enable(0.05)
    time.sleep(1.0)
    
    # 设置速度比例
    robot.set_speed_ratio(speed_ratio)
    print(f"速度比例设置为: {speed_ratio}%")
    time.sleep(0.5)
    
    # 检查机器人状态
    print(f"当前机器人状态: {robot.robot_mode}")
    
    if robot.robot_mode not in ['ENABLE', 'RUNNING']:
        print(f"警告: 机器人状态异常 - {robot.robot_mode}")
        response = input("是否继续执行? (y/n): ")
        if response.lower() != 'y':
            robot.disconnect()
            print("用户取消执行")
            return
    
    print()
    
    # ====== 执行轨迹 ======
    print("====== 开始执行扫描轨迹 (ServoJ模式) ======")
    
    # ServoJ 参数配置（固定值）
    servo_t = 0.01  # ServoJ指令周期 (s)
    servo_lookahead = 0.1  # 前瞻时间 (s)
    servo_gain = 300  # 增益
    
    print(f"ServoJ参数: t={servo_t:.3f}s, lookahead={servo_lookahead:.2f}s, gain={servo_gain}\n")
    
    # 记录起始时间
    start_time = time.time()
    
    # 创建实时图表
    if HAS_MATPLOTLIB and workspace_trajectory is not None:
        plt.ion()
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制计划轨迹
        ax.plot(workspace_trajectory[:, 0], workspace_trajectory[:, 1], 
                workspace_trajectory[:, 2], 'b--', linewidth=1.5, label='计划轨迹')
        
        # 实时位置点
        current_point, = ax.plot([], [], [], 'ro', markersize=12, label='当前位置')
        
        # 已执行轨迹
        executed_line, = ax.plot([], [], [], 'g-', linewidth=2, label='已执行轨迹')
        
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title('扫描轨迹执行进度 (ServoJ模式)')
        ax.legend()
        ax.grid(True)
        
        # 设置坐标轴范围
        ax.set_xlim([workspace_trajectory[:, 0].min() - 50, workspace_trajectory[:, 0].max() + 50])
        ax.set_ylim([workspace_trajectory[:, 1].min() - 50, workspace_trajectory[:, 1].max() + 50])
        ax.set_zlim([workspace_trajectory[:, 2].min() - 50, workspace_trajectory[:, 2].max() + 50])
        
        plt.draw()
        plt.pause(0.001)
        
        executed_positions = []
    
    # 先移动到起始位置
    print("移动到起始位置...")
    start_joints_deg = np.rad2deg(joint_trajectory[0, :]).tolist()
    robot.joint_mov_j(start_joints_deg)
    time.sleep(3.0)
    print("到达起始位置，开始ServoJ控制\n")
    
    # 使用ServoJ逐点执行轨迹
    try:
        for i in range(num_points):
            # 记录本次循环开始时间
            loop_start = time.time()
            
            # 获取目标关节角度（转换为度）
            target_joints = joint_trajectory[i, :]
            target_joints_deg = np.rad2deg(target_joints).tolist()
            
            # 使用ServoJ发送运动指令
            robot.servo_j(target_joints_deg, servo_t, servo_lookahead, servo_gain)
            
            # 显示进度（每10个点显示一次）
            if i % 10 == 0 or i == 0 or i == num_points - 1:
                current_joint_angles = robot.joint_angles
                current_pose = robot.cartesian_pose
                
                print(f"点 {i+1}/{num_points} ({(i+1)/num_points*100:.1f}%) - "
                      f"关节: [{', '.join([f'{x:.1f}' for x in current_joint_angles])}] deg")
                
                # 更新实时图表
                if HAS_MATPLOTLIB and workspace_trajectory is not None:
                    executed_positions.append(current_pose[:3])
                    exec_array = np.array(executed_positions)
                    
                    current_point.set_data([current_pose[0]], [current_pose[1]])
                    current_point.set_3d_properties([current_pose[2]])
                    
                    executed_line.set_data(exec_array[:, 0], exec_array[:, 1])
                    executed_line.set_3d_properties(exec_array[:, 2])
                    
                    plt.draw()
                    plt.pause(0.001)
            
            # 检查机器人状态
            if robot.robot_mode == 'ERROR':
                raise RuntimeError("机器人出现错误，停止执行！")
            
            # 等待直到本次循环时间达到 servo_t
            elapsed = time.time() - loop_start
            remaining = servo_t - elapsed
            if remaining > 0:
                time.sleep(remaining)
        
        # 停止ServoJ模式
        print("\n停止ServoJ控制...")
        robot.stop_move()
        time.sleep(0.5)
        
        # 记录总时间
        elapsed_time = time.time() - start_time
        
        print("\n====== 轨迹执行完成！ ======")
        print(f"总执行时间: {elapsed_time:.2f} 秒")
        print(f"轨迹点数: {num_points}")
        print(f"平均每点用时: {elapsed_time/num_points:.3f} 秒")
        
    except KeyboardInterrupt:
        print("\n用户中断执行")
        robot.stop_move()
    except Exception as e:
        print(f"\n执行出错: {e}")
        robot.stop_move()
    
    # ====== 返回原点（可选） ======
    response = input("\n是否返回零位？(y/n): ")
    if response.lower() == 'y':
        print("返回零位中...")
        robot.joint_mov_j([0, 0, 0, 0, 0, 0])
        time.sleep(3.0)
    
    # ====== 断开连接 ======
    print("\n====== 断开连接 ======")
    robot.disable()
    time.sleep(0.5)
    robot.disconnect()
    print("机器人已断开连接")
    
    print("\n====== 扫描任务完成！ ======")
    
    # 保持图表显示
    if HAS_MATPLOTLIB and workspace_trajectory is not None:
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    # 命令行参数处理
    if len(sys.argv) < 2:
        print("用法: python execute_scanning_trajectory.py <轨迹文件> [速度比例]")
        print("示例: python execute_scanning_trajectory.py scanning_trajectory_20251217_200823.mat 30")
        sys.exit(1)
    
    trajectory_file = sys.argv[1]
    speed_ratio = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    
    execute_scanning_trajectory(trajectory_file, speed_ratio)
