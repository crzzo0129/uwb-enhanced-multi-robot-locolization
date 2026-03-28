# 自定义UKF融合定位系统 - 完整使用指南

## 概述

本项目实现了一个从零开始的无损卡尔曼滤波(Unscented Kalman Filter, UKF)定位系统，用于融合多机器人的三个数据源：

1. **里程计 (Odometry)** `/rbX/odom` - 机器人自身的位置和速度估计
2. **UWB三角定位 (Trilateration)** `/rbX/tri_pos_in_rbY` - 多机器人间的相对位置测量
3. **IMU数据** (可选) - 用于增强航向角估计

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                     Multi-Robot Localization               │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Gazebo Simulator / Real Robots                            │
│       ├── rb1, rb2, rb3                                     │
│       └── Ground truth poses                                │
│                                                             │
│         ↓                                                    │
│                                                             │
│  Odometry & IMU                                             │
│       ├── /rb1/odom, /rb2/odom, /rb3/odom                  │
│       └── /rb1/imu, /rb2/imu, /rb3/imu                     │
│                                                             │
│         ↓                                                    │
│                                                             │
│  UWB距离测量 (robot_distance_plugin)                       │
│       ├── /robot_distances [d12, d13, d23]                │
│                                                             │
│         ↓                                                    │
│                                                             │
│  三角定位计算 (multi_robot_trilat)                         │
│       ├── /rb1/tri_pos_in_rb2  (rb2在rb1坐标系)             │
│       ├── /rb1/tri_pos_in_rb3  (rb3在rb1坐标系)             │
│       ├── /rb2/tri_pos_in_rb1  (rb1在rb2坐标系)             │
│       ├── /rb2/tri_pos_in_rb3  (rb3在rb2坐标系)             │
│       ├── /rb3/tri_pos_in_rb1  (rb1在rb3坐标系)             │
│       └── /rb3/tri_pos_in_rb2  (rb2在rb3坐标系)             │
│                                                             │
│         ↓                                                    │
│                                                             │
│  ┌─────────────────────────────────────────────┐           │
│  │     自定义 UKF 融合定位节点                 │           │
│  ├─────────────────────────────────────────────┤           │
│  │                                             │           │
│  │  predict(): 恒速运动模型                    │           │
│  │  updateOdom(): 融合里程计数据               │           │
│  │  updateTrilat(): 融合相对位置测量           │           │
│  │                                             │           │
│  └─────────────────────────────────────────────┘           │
│                                                             │
│         ↓                                                    │
│                                                             │
│  融合位置估计                                              │
│       ├── /rb1/ukf_pose  (融合估计的rb1位置)               │
│       ├── /rb2/ukf_pose  (融合估计的rb2位置)               │
│       └── /rb3/ukf_pose  (融合估计的rb3位置)               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## 安装与编译

### 1. 前置条件

```bash
# 安装Eigen3
sudo apt-get install libeigen3-dev

# 确保ROS2环境已配置
source /opt/ros/galactic/setup.bash
```

### 2. 编译

```bash
cd ~/fusion_ws
colcon build --packages-select robot_fusion
source install/setup.bash
```

## 运行系统

### 完整工作流（推荐顺序）

```bash
# 终端1: 启动Gazebo仿真环境
ros2 launch robot_fusion robot_fusion.launch.py

# 终端2: 启动三角定位节点(计算相对位置)
ros2 launch multi_robot_trilat trilat.launch.py

# 终端3: 启动自定义UKF融合节点
ros2 launch robot_fusion custom_ukf.launch.py

# 终端4 (可选): 给里程计添加噪声
ros2 launch odom_noise odom_noise.launch.py

# 终端5 (可选): 键盘控制机器人
ros2 launch robot_fusion teleop_robots.launch.py
```

## 配置文件说明

### `config/custom_ukf_rb1.yaml`

```yaml
/**:
  ros__parameters:
    # 输入话题
    odom_topic: '/rb1/odom'                    # 里程计话题
    trilat_topic_other1: '/rb1/tri_pos_in_rb2' # 相对位置：rb2相对rb1
    trilat_topic_other2: '/rb1/tri_pos_in_rb3' # 相对位置：rb3相对rb1
    
    # 输出话题
    output_topic: '/rb1/ukf_pose'
    
    # 更新频率
    update_rate: 20.0  # Hz
    
    # 测量噪声（标准差）
    odom_pos_noise: 0.01    # 里程计位置噪声 (m)
    odom_ang_noise: 0.001   # 里程计角度噪声 (rad)
    trilat_noise: 0.0225    # 三角定位噪声方差 (m²)
    
    # 过程噪声
    process_noise_pos: 0.01   # 位置过程噪声
    process_noise_ang: 0.001  # 角度过程噪声
```

### 参数调优

**位置估计精度**:
- 降低 `odom_pos_noise` 增加对里程计的信任
- 降低 `process_noise_pos` 使估计更平滑

**旋转估计精度**:
- 调整 `odom_ang_noise` 和 `process_noise_ang`
- 值越小对测量信任度越高

**融合效果**:
- 降低 `trilat_noise` 增加对三角定位数据的权重
- 增加 `process_noise_*` 降低模型信任度

## 数据可视化与验证

### 1. 查看当前估计

```bash
# 查看机器人1的UKF估计
ros2 topic echo /rb1/ukf_pose

# 查看原始里程计
ros2 topic echo /rb1/odom

# 查看三角定位结果
ros2 topic echo /rb1/tri_pos_in_rb2
```

### 2. 频率检查

```bash
# 检查UKF输出频率（应该是20 Hz）
ros2 topic hz /rb1/ukf_pose

# 检查输入话题频率
ros2 topic hz /rb1/odom
ros2 topic hz /rb1/tri_pos_in_rb2
```

### 3. 使用RViz可视化

```bash
# 启动RViz
rviz2

# 添加以下话题：
# - /rb1/odom (Odometry)
# - /rb1/ukf_pose (PoseWithCovarianceStamped)
# - /rb1/tri_pos_in_rb2 (PoseWithCovarianceStamped)

# 比较三种定位方式的差异
```

## UKF算法细节

### 状态向量 (6维)

$$\mathbf{x} = [x, y, \theta, v_x, v_y, \omega]^T$$

- $(x, y)$: 全局坐标系位置
- $\theta$: 航向角 (Yaw)
- $(v_x, v_y)$: 机器人坐标系速度
- $\omega$: 角速度

### 运动模型

```
x_{k+1} = f(x_k, u_k, dt)
```

**坐标变换**:
```cpp
// 机器人坐标系速度转全局坐标系
v_world_x = vx * cos(theta) - vy * sin(theta)
v_world_y = vx * sin(theta) + vy * cos(theta)

// 位置更新
x_{k+1} = x_k + v_world_x * dt
y_{k+1} = y_k + v_world_y * dt
theta_{k+1} = theta_k + omega * dt
```

### 测量模型

**1. 里程计测量**:
```
z_odom = h_odom(x) = [x, y, theta]^T
```
直接测量全局位置和航向

**2. 三角定位相对位置测量**:
```
z_trilat = h_trilat(x) = [x_rel, y_rel]^T
```
测量其他机器人的相对位置（在全局坐标系）

### Sigma点生成

无损变换通过生成 $2n+1$ 个代表性点来近似非线性传播：

```
χ^{(0)} = x_k

χ^{(i)} = x_k + √(n + λ) * L_k[:,i],  i = 1...n

χ^{(i)} = x_k - √(n + λ) * L_k[:,i-n],  i = n+1...2n
```

其中 $L_k$ 是协方差矩阵的Cholesky分解

### 权重计算

```
λ = α²(n + κ) - n  = 1e-6 * 7 - 6 ≈ 0
Wm[0] = λ / (n + λ)
Wc[0] = λ / (n + λ) + (1 - α² + β)
Wm[i] = Wc[i] = 1 / (2(n + λ))  for i = 1...2n
```

其中:
- $\alpha = 1e-3$ (Sigma点展开度)
- $\beta = 2.0$ (高斯分布最优值)
- $\kappa = 0$ (二级参数)

## 性能评估

### 评估指标

1. **位置误差 (RMSE)**
   ```
   RMSE_pos = √(mean((x_est - x_true)² + (y_est - y_true)²))
   ```

2. **角度误差**
   ```
   RMSE_ang = √(mean((θ_est - θ_true)²))
   ```

3. **一致性检验 (NEES)**
   ```
   NEES = Σ(x_err)^T * P^{-1} * (x_err)
   ```
   应该接近状态维数(6)

### 评估脚本

```bash
# 启动评估节点
ros2 launch trilat_eval eval_circle.launch.py

# 该节点会：
# 1. 记录真实值、里程计、三角定位、UKF估计
# 2. 计算各自的误差
# 3. 生成对比图表
```

## 常见问题与故障排除

### Q1: UKF估计发散

**症状**: 估计值越来越远离真实值

**检查清单**:
```bash
# 1. 检查里程计数据质量
ros2 topic echo /rb1/odom
# 确保位置值合理变化，不要有跳跃

# 2. 检查三角定位数据
ros2 topic echo /rb1/tri_pos_in_rb2
# 确保相对位置值在合理范围内

# 3. 调整噪声参数
# 在config/custom_ukf_rb1.yaml中：
# - 增加 process_noise_pos 和 process_noise_ang
# - 减少对测量的信任
```

**解决方案**:
```yaml
# 更保守的初始参数
odom_pos_noise: 0.02      # 增加
process_noise_pos: 0.05   # 增加
trilat_noise: 0.1         # 增加
```

### Q2: 三角定位数据未被使用

**症状**: 查看`/rb1/ukf_pose`，似乎只依赖于里程计

**检查清单**:
```bash
# 1. 确认三角定位节点运行
ros2 launch multi_robot_trilat trilat.launch.py

# 2. 检查话题存在
ros2 topic list | grep tri_pos_in_rb

# 3. 检查话题是否有消息
ros2 topic hz /rb1/tri_pos_in_rb2
# 应该输出约20Hz的频率

# 4. 查看消息内容
ros2 topic echo /rb1/tri_pos_in_rb2
```

**解决方案**:
```bash
# 如果话题不存在或无消息，检查trilat.launch.py配置
# 确保节点参数正确指向各机器人
```

### Q3: 估计滞后于实时位置

**症状**: UKF输出的位置明显滞后于机器人实际位置

**原因**: 
- 过程噪声过高 (对模型不信任)
- 测量噪声过低 (过度依赖历史估计)
- 更新频率过低

**解决方案**:
```yaml
# 降低过程噪声，增加对模型的信任
process_noise_pos: 0.005  # 降低
process_noise_ang: 0.0005 # 降低

# 增加更新频率
update_rate: 30.0  # 从20提高到30

# 相对降低测量噪声
odom_pos_noise: 0.005
```

### Q4: 角度估计不稳定

**症状**: 航向角 (yaw) 抖动或偏差大

**原因**:
- 里程计的角度噪声设置不合理
- 缺少IMU数据

**解决方案**:
```yaml
# 增加对角度的信任
odom_ang_noise: 0.0005  # 降低
process_noise_ang: 0.0002 # 降低

# 或增加过程噪声（模型不确定性）
process_noise_ang: 0.002
```

## 高级使用

### 1. 添加IMU测量

修改 `src/robot_ukf_node.cpp`，增加IMU订阅:

```cpp
private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

// 在构造函数中
imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/rb" + robot_name_.back() + "/imu", 10,
    std::bind(&RobotUKFNode::imuCallback, this, std::placeholders::_1));

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // 从IMU四元数提取角速度和加速度
    // 用于增强航向角和加速度估计
}
```

### 2. 异常值拒绝

增加马氏距离检验:

```cpp
void RobotUKFNode::updateWithOutlierRejection(const Eigen::Vector3d& z) {
    // 计算预测和实际的差异
    Eigen::Vector3d innovation = z - z_pred;
    
    // 马氏距离
    double mahal = innovation.transpose() * Pz.inverse() * innovation;
    
    // 设定阈值（自由度=3）
    if (mahal > 7.81) {  // 99% 置信度
        RCLCPP_WARN(get_logger(), "Outlier rejected: mahal=%.2f", mahal);
        return;
    }
    
    // 正常更新
    ukf_->updateOdom(z);
}
```

### 3. 自适应噪声

根据残差动态调整噪声参数:

```cpp
void adaptiveNoiseAdjustment(const std::vector<Eigen::Vector3d>& residuals) {
    double mean_residual = 0;
    for (const auto& r : residuals) {
        mean_residual += r.norm();
    }
    mean_residual /= residuals.size();
    
    // 如果残差增大，增加过程噪声
    if (mean_residual > expected_residual * 1.5) {
        Q_ *= 1.1;  // 增加10%
    } else if (mean_residual < expected_residual * 0.5) {
        Q_ *= 0.95; // 降低5%
    }
}
```

## 性能优化建议

1. **增加状态向量维度** - 如果需要估计加速度或更复杂的运动
2. **多假设跟踪** - 处理三角定位的镜像解
3. **图优化** - 闭环时使用全局优化
4. **粒子滤波** - 处理多模态分布

## 文件结构

```
robot_fusion/
├── CMakeLists.txt                      # 构建配置
├── package.xml                         # 包信息和依赖
├── README_UKF.md                       # 详细技术说明
│
├── include/
│   └── robot_fusion/
│       └── ukf.hpp                     # UKF库头文件
│
├── src/
│   ├── ukf.cpp                         # UKF库实现
│   ├── robot_ukf_node.cpp              # ROS2节点
│   ├── distance_to_pose_wrapper.cpp    # 距离转相对位置
│   └── robot_distance_plugin.cpp       # Gazebo插件
│
├── config/
│   ├── custom_ukf_rb1.yaml             # rb1配置
│   ├── custom_ukf_rb2.yaml             # rb2配置
│   ├── custom_ukf_rb3.yaml             # rb3配置
│   ├── ukf_rb1.yaml                    # robot_localization配置
│   ├── ukf_rb2.yaml
│   └── ukf_rb3.yaml
│
├── launch/
│   ├── custom_ukf.launch.py            # 自定义UKF启动脚本
│   ├── ukf_fusion.launch.py            # robot_localization启动脚本
│   ├── robot_fusion.launch.py          # 仿真环境
│   └── ...
│
└── models/                             # Gazebo模型文件
    └── ...
```

## 参考资源

- Simon, D. (2006). Optimal State Estimation: Kalman, H∞, and Nonlinear Approaches
- Wan, E. A., & Van Der Merwe, R. (2000). The unscented Kalman filter for nonlinear estimation
- [ROS2官方文档](https://docs.ros.org/)
- [Eigen3文档](https://eigen.tuxfamily.org/)

## 许可与支持

如有问题，请检查日志输出和话题数据。
