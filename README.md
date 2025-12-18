# Dobot CR5 机械臂相机扫描轨迹系统

> 用于控制 Dobot CR5 机械臂末端相机进行圆周扫描的完整系统，支持 MATLAB 和 Python 两种环境。

---

## 📁 项目文件结构

### 核心文件

#### MATLAB 代码
- **`ZJFDobotCR5.m`** - 机器人通信控制类（TCP/IP，Dashboard/Move/Feedback 三端口）
- **`create_dobot_cr5_model.m`** - 创建机器人模型（基于 Robotics Toolbox）
- **`plan_scanning_trajectory.m`** - 轨迹规划脚本（圆周扫描，IK 求解）
- **`execute_scanning_trajectory.m`** - 轨迹执行脚本（ServoJ 平滑控制）

#### Python 代码
- **`dobot_cr5.py`** - 机器人通信类（多线程实时反馈）
- **`execute_scanning_trajectory.py`** - 轨迹执行脚本（ServoJ，3D 可视化）
- **`requirements.txt`** - Python 依赖包
- **`README_Python.md`** - Python 版本详细说明

#### 配置与数据
- **`DH_table.txt`** - DH 参数表
- **`scanning_trajectory_*.mat`** - 轨迹数据文件

---

## 🚀 快速开始

### 方案 A：MATLAB 规划 + MATLAB 执行

```matlab
% 1. 规划轨迹
plan_scanning_trajectory

% 2. 执行轨迹
execute_scanning_trajectory('scanning_trajectory_20251217_225246.mat', 30)
```

### 方案 B：MATLAB 规划 + Python 执行 ✨ 推荐

```bash
# 1. MATLAB 中规划轨迹
# plan_scanning_trajectory

# 2. Python 执行（速度比例 30%）
python execute_scanning_trajectory.py scanning_trajectory_20251217_225246.mat 30
```

---

## 📚 详细使用说明

### 1️⃣ 轨迹规划（MATLAB）

#### 安装依赖
```matlab
% 需要 Peter Corke's Robotics Toolbox for MATLAB
% 下载: https://petercorke.com/toolboxes/robotics-toolbox/
```

#### 配置参数

在 `plan_scanning_trajectory.m` 中修改（第 9-25 行）：

```matlab
% 目标物体位置
target_position = [-470, -143, 50];  % [X, Y, Z] mm

% 圆周扫描参数
scan_radius = 200;          % 扫描半径 (mm)
scan_height = 300;          % 扫描高度 (mm)
camera_tilt = 15;           % 相机俯视角度 (度)
num_rotations = 1;          % 扫描圈数

% 速度和时间参数
scan_velocity = 60;         % 扫描速度 (mm/s)
servo_t = 0.01;             % 轨迹点时间间隔 (s)
```

#### 运行规划

```matlab
plan_scanning_trajectory
```

**输出**：
- 轨迹文件：`scanning_trajectory_YYYYMMDD_HHMMSS.mat`
- 可视化图表：3D 轨迹、关节角度、速度剖面等

**保存数据**：
- `joint_trajectory` - 关节角度序列（弧度）
- `workspace_trajectory` - 笛卡尔位姿序列
- `velocities` - 速度剖面
- `timestamps` - 时间戳
- `servo_t` - 时间间隔参数

---

### 2️⃣ 轨迹执行

#### 选项 A：MATLAB 执行

```matlab
% 基本用法（默认速度 30%）
execute_scanning_trajectory('scanning_trajectory_20251217_225246.mat')

% 指定速度比例
execute_scanning_trajectory('scanning_trajectory_20251217_225246.mat', 50)
```

**特点**：
- ✅ ServoJ 平滑控制
- ✅ 实时 3D 可视化
- ✅ 固定 0.01s 时间间隔
- ✅ 自动时间补偿

#### 选项 B：Python 执行 ✨

**安装依赖**：
```bash
pip install -r requirements.txt
# 需要：numpy, scipy, matplotlib
```

**基本用法**：
```bash
python execute_scanning_trajectory.py scanning_trajectory_20251217_225246.mat 30
```

**特点**：
- ✅ 与 MATLAB 版本功能完全一致
- ✅ 读取 MAT 文件（scipy.io）
- ✅ 多线程实时反馈
- ✅ 3D 轨迹可视化
- ✅ 性能优异（MAT 读取 <50ms）

完整 Python 说明请参阅：[`README_Python.md`](README_Python.md)

---

## ⚙️ 核心参数说明

### 轨迹规划参数

| 参数 | 说明 | 单位 | 推荐值 |
|------|------|------|--------|
| `scan_velocity` | 扫描速度 | mm/s | 20-100 |
| `servo_t` | 点间时间间隔 | s | 0.01 |
| `scan_radius` | 扫描半径 | mm | 100-300 |
| `scan_height` | 扫描高度 | mm | 200-400 |
| `camera_tilt` | 俯视角度 | deg | 5-30 |
| `num_rotations` | 扫描圈数 | - | 1-3 |

### ServoJ 参数

固定配置（在执行脚本中）：

```matlab/python
servo_t = 0.01          % 指令周期 (s)
servo_lookahead = 0.1   % 前瞻时间 (s)
servo_gain = 300        % 增益
```

**调整建议**：
- 更平滑：增大 `servo_lookahead` → 0.15-0.2s
- 更快响应：提高 `servo_gain` → 400-500
- 更高频率：减小 `servo_t` → 0.008s（需相应调整规划）

---

## 📊 轨迹数据格式

MAT 文件变量：

```matlab
workspace_trajectory    % [N×6] 笛卡尔轨迹 [X,Y,Z,Rx,Ry,Rz]
joint_trajectory        % [N×6] 关节轨迹 (弧度)
velocities             % [N×1] 速度剖面 (mm/s)
timestamps             % [N×1] 时间戳 (s)
servo_t                % 标量  时间间隔 (s)
scan_velocity          % 标量  扫描速度 (mm/s)
total_scan_time        % 标量  总时间 (s)
target_position        % [1×3] 目标位置
scan_radius            % 标量  扫描半径
scan_height            % 标量  扫描高度
```

---

## 🔧 机器人连接参数

默认配置（可在代码中修改）：

```matlab
IPAddress = '192.168.5.1'
DashboardPort = 29999
MovePort = 30003
FeedbackPort = 30004
```

---

## 💡 使用示例

### 示例 1：标准扫描

```matlab
% 1. 打开规划脚本，设置参数
scan_velocity = 40;      % 中速
scan_radius = 200;       % 标准半径
num_rotations = 1;       % 单圈

% 2. 运行规划
plan_scanning_trajectory

% 3. 执行（MATLAB）
execute_scanning_trajectory('scanning_trajectory_20251217_225246.mat', 30)

% 或执行（Python）
% python execute_scanning_trajectory.py scanning_trajectory_20251217_225246.mat 30
```

### 示例 2：快速多圈扫描

```matlab
% 规划参数
scan_velocity = 80;      % 高速
num_rotations = 2;       % 双圈
servo_t = 0.008;         % 更高频率

% 规划并执行
plan_scanning_trajectory
execute_scanning_trajectory('scanning_trajectory_XXXXXX.mat', 50)
```

### 示例 3：精细慢速扫描

```matlab
% 规划参数
scan_velocity = 20;      % 低速
camera_tilt = 10;        % 小角度
servo_t = 0.01;

% 规划并执行
plan_scanning_trajectory
execute_scanning_trajectory('scanning_trajectory_XXXXXX.mat', 20)
```

---

## ⚠️ 注意事项

### 安全警告
- ⚠️ **首次执行前务必在安全环境测试**
- ⚠️ **确保机械臂周围无障碍物**
- ⚠️ **建议从低速（20-30%）开始**
- ⚠️ **准备好急停装置**

### 轨迹质量检查
1. 查看可视化图表确认轨迹合理
2. 检查关节角度是否在限位内（第 2 个图）
3. 查看速度剖面是否平滑（第 4 个图）
4. 注意 IK 误差警告

### 执行前检查
1. 机器人状态正常（ENABLE 或 RUNNING）
2. 负载参数正确（默认 0.05kg）
3. 速度比例合理（建议 30-50%）
4. 轨迹文件加载成功

---

## 🐛 故障排除

### IK 求解问题
**症状**：出现 IK 误差警告或求解失败
**解决**：
- 减小扫描范围（半径、高度）
- 调整目标位置到工作空间中心
- 检查相机俯视角不要太大

### 连接失败
**症状**：无法连接机器人
**解决**：
- 检查 IP 地址（`ipconfig` 查看网络）
- 确认机器人控制器开启
- 防火墙允许端口通信

### 运动不平滑
**症状**：机械臂运动抖动
**解决**：
- 降低扫描速度 `scan_velocity`
- 增加 `servo_lookahead` 参数
- 检查网络延迟

### Python 可视化问题
**症状**：matplotlib 报错或不显示
**解决**：
```bash
# Linux 可能需要
export DISPLAY=:0

# 或禁用可视化（修改代码 HAS_MATPLOTLIB = False）
```

---

## 🔬 技术规格

| 项目 | MATLAB 版本 | Python 版本 |
|------|------------|------------|
| **编程语言** | MATLAB R2019b+ | Python 3.7+ |
| **机器人模型** | Robotics Toolbox | - |
| **IK 求解** | ikcon/ikine | - |
| **通信协议** | TCP/IP | TCP/IP (socket) |
| **实时反馈** | Callback | Threading |
| **可视化** | MATLAB Plot | Matplotlib |
| **数据格式** | MAT | MAT (scipy.io) |
| **控制模式** | ServoJ | ServoJ |
| **控制频率** | 100Hz | 100Hz |

---

## 📖 相关文档

- [`README_Python.md`](README_Python.md) - Python 版本详细说明
- [Peter Corke's Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/) - MATLAB 运动学库
- [Dobot CR5 用户手册](https://www.dobot.cc/) - 机器人官方文档

---

## 🤝 贡献与支持

### 环境要求

**MATLAB**：
- MATLAB R2019b 或更高版本
- Robotics System Toolbox
- Optimization Toolbox

**Python**：
- Python 3.7+
- numpy >= 1.20.0
- scipy >= 1.7.0
- matplotlib >= 3.3.0

### 版本历史

- **v2.0** (2025-12-18)
  - ✨ 新增 Python 完整实现
  - ✨ ServoJ 平滑控制替代 JointMovJ
  - ✨ 速度参数自动计算
  - ✨ 增强的可视化和统计

- **v1.0** (2025-12-17)
  - 初始 MATLAB 版本
  - 基础轨迹规划和执行

---

## 📞 联系方式

如有问题或建议，请联系项目维护者。

**常见问题快速检查**：
1. MATLAB/Python 版本是否符合要求
2. 依赖库是否正确安装
3. 机器人网络连接是否正常
4. 轨迹参数是否在合理范围内

---

*最后更新：2025-12-18*
