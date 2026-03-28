# 自定义UKF融合定位系统

本实现提供了一个从零开始的无损卡尔曼滤波(Unscented Kalman Filter, UKF)定位系统，用于融合多机器人的里程计数据和三角定位测量。

## 系统架构

### 数据流向

```
Odometry (/rb1/odom)
    ↓
    ├→ UKF Prediction (恒速运动模型)
    │
    ├→ UKF Update (里程计测量)
    │
    ↓
Trilateration (/rb1/tri_pos_in_rb2, /rb1/tri_pos_in_rb3)
    ↓
    └→ UKF Update (相对位置测量)
    
    ↓
Estimated Pose (/rb1/ukf_pose)
```

## 核心模块

### 1. UKF 核心库 (`include/robot_fusion/ukf.hpp` 和 `src/ukf.cpp`)

实现了完整的无损卡尔曼滤波器，包括：

**状态向量 (6维):**
```
x = [x, y, θ, vx, vy, ω]^T
```
- `(x, y)`: 全局坐标系中的位置
- `θ`: 航向角 (yaw)
- `(vx, vy)`: 机器人坐标系中的速度
- `ω`: 角速度

**运动模型 (恒速模型):**
- 将机器人坐标系速度转换到全局坐标系
- 更新位置和航向角
- 速度和角速度随时间略有阻尼衰减

**测量模型:**

1. **里程计测量**: `z_odom = [x, y, θ]^T`
   - 直接测量全局位置和航向
   
2. **三角定位测量**: `z_trilat = [x_rel, y_rel]^T`
   - 测量其他机器人的相对位置

### 2. UKF 节点 (`src/robot_ukf_node.cpp`)

ROS2 节点，执行以下功能：

- **订阅话题:**
  - `/rbX/odom`: 里程计数据
  - `/rbX/tri_pos_in_rbY`: 三角定位相对位置测量

- **发布话题:**
  - `/rbX/ukf_pose`: 融合后的位置估计

- **处理流程:**
  1. 初始化: 使用第一次里程计数据初始化UKF
  2. 预测: 每个周期执行一次运动模型预测
  3. 更新: 
     - 里程计数据到达时更新
     - 三角定位数据到达时更新

## 关键算法

### 无损变换 (Unscented Transform)

1. **Sigma点生成:**
   - 从当前估计的均值和协方差生成 2n+1 个sigma点
   - 通过Cholesky分解计算平方根项

2. **权重计算:**
   ```
   α = 1e-3  (控制sigma点展开程度)
   β = 2.0   (高斯分布的最优值)
   κ = 0.0   (二级缩放参数)
   λ = α²(n+κ) - n
   ```

3. **传播和更新:**
   - 通过非线性模型传播所有sigma点
   - 使用加权平均计算新的均值
   - 计算新的协方差矩阵

### 角度处理

- 所有角度归一化到 [-π, π] 范围
- 角度差异使用专门的角度错误函数处理

## 配置参数

在 `config/custom_ukf_rbX.yaml` 中配置：

```yaml
odom_topic: '/rbX/odom'                    # 里程计输入话题
trilat_topic_other1: '/rbX/tri_pos_in_rbY' # 相对位置话题1
trilat_topic_other2: '/rbX/tri_pos_in_rbZ' # 相对位置话题2

update_rate: 20.0                          # 更新频率 (Hz)

odom_pos_noise: 0.01                       # 里程计位置噪声标准差
odom_ang_noise: 0.001                      # 里程计角度噪声标准差
trilat_noise: 0.0225                       # 三角定位噪声方差

process_noise_pos: 0.01                    # 过程噪声(位置)
process_noise_ang: 0.001                   # 过程噪声(角度)
```

## 使用说明

### 1. 编译

```bash
cd ~/fusion_ws
colcon build --packages-select robot_fusion
```

### 2. 运行系统

**首先启动仿真环境和三角定位节点:**
```bash
ros2 launch robot_fusion robot_fusion.launch.py
ros2 launch multi_robot_trilat trilat.launch.py
```

**然后启动自定义UKF:**
```bash
ros2 launch robot_fusion custom_ukf.launch.py
```

### 3. 查看结果

查看融合后的位置估计：
```bash
ros2 topic echo /rb1/ukf_pose
```

## 改进建议

### 1. 增强运动模型

当前使用恒速模型，可以改进为：

```cpp
// 使用轨迹曲率估计
double v = sqrt(vx*vx + vy*vy);
double curvature = omega / (v + 1e-9);
// 双自行车模型或常曲率圆弧模型
```

### 2. 自适应噪声

根据测量历史动态调整噪声参数：

```cpp
// 如果连续多次测量偏差大，增加过程噪声
// 如果测量一致性好，降低测量噪声
```

### 3. 异常值拒绝

```cpp
// 计算马氏距离
double mahal_dist = innovation.transpose() * Pz.inverse() * innovation;
if (mahal_dist > threshold) {
    // 拒绝异常测量
    return;
}
```

### 4. 多假设跟踪

当三角定位有多个解时（镜像解），可以使用多假设来跟踪：

```cpp
std::vector<UKF> particle_filters;  // 多个假设
// 定期验证和合并相近的假设
```

### 5. 闭环优化

如果机器人路径可以闭合：

```cpp
// 检测回到相同位置
// 使用全局优化(图优化)调整整个轨迹
```

## 性能指标

优化时关注以下指标：

1. **位置误差** (Position Error)
   - 相对于真实值的均方根误差 (RMSE)
   
2. **一致性** (Consistency)
   - 预测协方差与实际误差的一致性
   - 应满足: actual_error² ≈ predicted_covariance
   
3. **实时性** (Real-time Performance)
   - 每个周期的计算时间

## 故障排除

### 问题: UKF发散

**症状:** 估计位置越来越远离真实值

**解决:**
1. 检查里程计数据的噪声参数 (odom_pos_noise, odom_ang_noise)
2. 增加过程噪声参数
3. 检查运动模型是否与实际机器人行为匹配

### 问题: 三角定位数据未被使用

**症状:** `/rb1/tri_pos_in_rb2` 话题没有收到消息

**解决:**
1. 检查三角定位节点是否运行: `ros2 topic list`
2. 验证话题名称是否与配置文件匹配
3. 检查消息发布频率: `ros2 topic hz /rb1/tri_pos_in_rb2`

### 问题: 估计值滞后

**症状:** UKF输出的位置明显滞后于实时的里程计

**解决:**
1. 增加update_rate
2. 减少测量噪声方差(提高对测量的信任)
3. 检查计算时间是否过长

## 文件列表

```
robot_fusion/
├── include/
│   └── robot_fusion/
│       └── ukf.hpp                 # UKF核心头文件
├── src/
│   ├── ukf.cpp                     # UKF核心实现
│   └── robot_ukf_node.cpp          # ROS2节点
├── config/
│   ├── custom_ukf_rb1.yaml         # 机器人1配置
│   ├── custom_ukf_rb2.yaml         # 机器人2配置
│   └── custom_ukf_rb3.yaml         # 机器人3配置
├── launch/
│   └── custom_ukf.launch.py        # 启动脚本
├── CMakeLists.txt                  # 构建配置
└── package.xml                     # 包依赖声明
```

## 参考文献

1. Simon, D. (2006). Optimal State Estimation: Kalman, H∞, and Nonlinear Approaches
2. Wan, E. A., & Van Der Merwe, R. (2000). The unscented Kalman filter for nonlinear estimation
3. Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). Estimation with Applications to Tracking and Navigation

## 许可证

TODO: License declaration
