# Python 轨迹执行脚本使用说明

## 📁 文件说明

- **`dobot_cr5.py`**: Dobot CR5 机器人通信类
- **`execute_scanning_trajectory.py`**: 轨迹执行脚本
- **`requirements.txt`**: Python 依赖包

## 🔧 安装

### 1. 安装 Python 依赖

```bash
pip install -r requirements.txt
```

### 2. 验证安装

```bash
python -c "import numpy, scipy, matplotlib; print('All dependencies installed!')"
```

## 🚀 使用方法

### 基本用法

```bash
python execute_scanning_trajectory.py <轨迹文件.mat> [速度比例]
```

### 示例

```bash
# 使用默认速度比例 (30%)
python execute_scanning_trajectory.py scanning_trajectory_20251217_200823.mat

# 指定速度比例为 50%
python execute_scanning_trajectory.py scanning_trajectory_20251217_200823.mat 50
```

## 📊 功能对比

| 功能 | MATLAB 版本 | Python 版本 |
|------|------------|------------|
| 加载轨迹文件 | ✅ | ✅ |
| TCP/IP 通信 | ✅ | ✅ |
| ServoJ 控制 | ✅ | ✅ |
| 实时反馈 | ✅ | ✅ (多线程) |
| 可视化 | ✅ | ✅ (matplotlib) |
| 速度控制 | ✅ | ✅ |
| 错误处理 | ✅ | ✅ |

## 🎯 主要特性

### 1. ServoJ 平滑控制
- 固定时间间隔 (默认 0.01s)
- 自动时间补偿
- 连续平滑运动

### 2. 实时状态监控
- 关节角度实时反馈
- 笛卡尔位姿显示
- 机器人状态检测

### 3. 可视化
- 3D 轨迹显示
- 实时执行进度
- 已执行路径跟踪

## ⚙️ 参数说明

### ServoJ 参数 (在脚本中配置)

```python
servo_t = 0.01          # 指令周期 (秒)
servo_lookahead = 0.1   # 前瞻时间 (秒)
servo_gain = 300        # 增益
```

### 机器人连接参数

默认配置（可在 `dobot_cr5.py` 中修改）:
```python
ip_address = '192.168.5.1'
dashboard_port = 29999
move_port = 30003
feedback_port = 30004
```

## 🛠️ 故障排除

### 1. 连接失败
- 检查机器人 IP 地址
- 确认网络连接
- 验证端口未被占用

### 2. 导入错误
```bash
# 重新安装依赖
pip install --upgrade -r requirements.txt
```

### 3. 可视化不显示
- 检查是否安装了 matplotlib
- Linux 系统可能需要配置 X11

## 📝 代码示例

### 自定义使用

```python
from dobot_cr5 import DobotCR5
import numpy as np

# 创建机器人对象
robot = DobotCR5(ip_address='192.168.5.1')

# 连接
robot.connect()

# 使能
robot.enable(0.05)

# ServoJ 控制
joints = [0, 30, -60, 0, 90, 0]  # 度
robot.servo_j(joints, t=0.01, lookahead_time=0.1, gain=300)

# 断开
robot.disconnect()
```

## ⚡ 性能说明

- **控制频率**: 100Hz (servo_t = 0.01s)
- **通信延迟**: < 10ms
- **实时反馈**: 多线程异步接收
- **可视化更新**: 每10个点

## 🔄 与 MATLAB 版本的差异

1. **依赖库**: 使用 scipy 读取 .mat 文件
2. **多线程**: 使用 threading 模块处理实时反馈
3. **可视化**: matplotlib 替代 MATLAB 绘图
4. **错误处理**: Python 异常处理机制

## 📞 支持

如有问题，请检查:
1. 机器人连接状态
2. Python 版本 (推荐 3.7+)
3. 依赖包版本
4. 轨迹文件格式
