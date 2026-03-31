> [English](README.md) | 🌐 **简体中文**

---

# UWB增强的多机器人协同定位系统

![eval_result](./assets/eval_result.png)

**基于 UWB 的多机器人协同定位系统** *(ROS 2 + Gazebo 仿真)*

本项目实现了一个利用**超宽带 (UWB)** 测距数据来提升多机器人**相对定位**精度的框架。通过将每个机器人的轮式里程计与机器人间的 UWB 距离观测数据进行融合，该系统在无 GNSS 环境（如室内、地下或密林）中大幅抑制了定位漂移。

核心技术包括：
- 用于相对位置估计的多机器人多边测距 (Trilateration)
- 用于传感器融合的无迹卡尔曼滤波 (UKF)
- 包含自定义 UWB 测距插件的高逼真 Gazebo 仿真环境

---

## 🔬 背景与应用场景

传统仅依赖里程计、IMU 和 Lidar/Visual SLAM 的无迹卡尔曼滤波 (UKF) 算法，先天存在累积漂移和对运动学连续性的强依赖。当系统遭遇极端空间状态突变或持续的几何特征缺失时，标准的状态估计会发生发散，导致不可逆的定位崩溃。通过融合超宽带 (UWB) 提供的绝对距离约束，本多机器人协同定位系统利用节点间的空间刚性约束，从根本上克服了这些局限性。

<details>
<summary><b>场景一：极端状态突变与全局重定位（绑架机器人问题）</b></summary>

https://github.com/user-attachments/assets/c812a918-5a30-4f5a-aba6-89a2956db225

* **背景痛点：** 传统 Lidar/Visual SLAM 强依赖连续的运动学平滑推演。当机器人遭遇“绑架”（人工搬运、严重物理撞击或履带彻底打滑）时，连续位姿假设破裂，扫描匹配失效，状态协方差瞬间发散，导致定位与建图系统彻底崩溃。
* **底层逻辑：** UWB 提供了不受运动学连续性限制的绝对距离约束。当目标车辆发生非连续的大尺度空间位移时，UKF 绕过失效的 Lidar 前端，直接利用其余两台协同车辆的高置信度位置先验，通过 UWB 多边测距的量测方程，在极短的滤波周期内将发散的位姿强制收敛至真实的物理坐标，实现**无特征依赖的全局重定位**。
* **前提假设与潜在风险：** 本场景假设作为参考锚点的另外两台车辆在突变期间维持精确的全局位姿。潜在风险在于，若绑架跨度直接超出了 UWB 的物理最大通信半径，或遭遇严重的射频屏蔽，重定位推演将无法完成。
</details>

<details>
<summary><b>场景二：几何特征退化环境的动态补偿</b></summary>

https://github.com/user-attachments/assets/6a4bd67f-8c1b-4a10-94bc-a861f62946be

* **背景痛点：** 在长直走廊或空旷场地中，激光雷达在特定自由度（如纵向前进方向）上会遭遇几何特征缺失。点云配准算法因此发生退化，导致底盘里程计的积分漂移不受控制，生成的地图出现严重的纵向拉伸或压缩。
* **底层逻辑：** 在该退化状态下，远端的两台协同车辆构成了动态的 UWB 基准网络。UKF 算法动态调整传感器权重，将 UWB 提供的低频、绝对距离约束注入状态更新阶段。这一机制精准补齐了 Lidar 在单一自由度上的观测盲区，利用相对距离的刚性约束，物理级锁死了由特征缺失带来的纵向累积漂移。
* **前提假设与潜在风险：** 假设辅助定位的车辆位于非退化区域或具备独立可靠的定位源。一个显著的潜在风险是**共线模糊 (Collinearity Ambiguity)**：如果三台小车在走廊中完全处于同一直线上，UWB 测距在侧向（横向）的几何约束力降为零，这可能导致横向协方差估计出现矩阵奇异或翻转模糊。
</details>

## ✨ 核心特性

- **无基站相对定位**：机器人在无需外部物理锚点或全局基准的情况下，在各自的局部坐标系中估算彼此的位置。
- **鲁棒的传感器融合**：利用 UKF 将含有噪声的里程计数据与 UWB 测距数据进行深度融合。
- **模块化架构**：
  - `multi_robot_trilat/`: 多机器人多边测距节点
  - `robot_fusion/`: UKF 融合、Gazebo 仿真及 UWB 距离发布器
  - `odom_noise/`: 高逼真里程计噪声注入仿真
  - `trilat_eval/`: 定位精度评估工具（与 Ground Truth 对比计算）
- **开箱即用的仿真**：原生支持 3 台及以上机器人的协同仿真，支持手动控制圆周运动以进行算法评估。
- **ROS 2 原生支持**：采用标准消息接口，极易无缝迁移至真实物理硬件。

---

## 📁 代码库结构

```text
uwb-enhanced-multi-robot-locolization/
├── multi_robot_trilat/          # 多机器人多边测距计算模块
├── odom_noise/                  # 里程计噪声注入模块
├── robot_fusion/                # UKF融合 + Gazebo环境 + UWB插件
│   ├── launch/                  # 启动文件
│   └── ...
├── trilat_eval/                 # 精度评估与指标计算工具
├── uwbpsr_ratros2/              # UWB相关ROS 2底层驱动包（如有）
├── LICENSE
├── README.md
└── instructions.txt             # 原始中文运行指令备份
```

---

## 🚀 快速上手

<details>
  <summary><b>环境要求与使用方式</b></summary

### 环境要求

- **操作系统**: Ubuntu 20.04 或 22.04
- **ROS 2 版本**: Galactic (推荐) 或 Iron
- **Gazebo 版本**: Harmonic 或 Classic (取决于 ROS 2 版本)
- **编译工具**: colcon, rosdep

###编译与安装

```bash
# 1. 创建工作空间
mkdir -p ~/fusion_ws/src
cd ~/fusion_ws/src

# 2. 克隆代码仓库
git clone [https://github.com/crzzo0129/uwb-enhanced-multi-robot-locolization.git](https://github.com/crzzo0129/uwb-enhanced-multi-robot-locolization.git)

# 3. 安装依赖项
cd ~/fusion_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. 编译工作空间
colcon build --symlink-install
source install/setup.bash
```

### 运行仿真
详细的启动命令请参考根目录下的 指令.txt。典型工作流如下：

```bash
# 启动包含 UWB 与 UKF 的完整多机器人仿真系统（示例）
ros2 launch robot_fusion robot_fusion.launch.py
ros2 launch odom_noise odom_noise.launch.py
ros2 launch robot_fusion custom_ukf.launch.py
```

**提示**: 在手动遥控机器人进行圆周运动时，可同步启动 trilat_eval 模块收集数据，以进行定位精度与误差评估。

---

## 📊 系统架构总览
1. **UWB 测距仿真**: 自定义 Gazebo 插件模拟真实的无线电噪声，并发布机器人间的距离数据（如 `/robot_distances` 话题）。

2. **多边测距定位**: `multi_robot_trilat` 节点利用 UWB 距离计算出粗略的相对位置（输出类似 `/rbX/tri_pos_in_rbY` 的话题）。

3. **UKF 滤波融合**: 每台机器人运行独立的 UKF 节点，将其自身的里程计流与多边测距观测结果进行融合，输出平滑且高精度的位姿估计。

4. **噪声建模建模**: `odom_noise` 模块向 Gazebo 输出的理想里程计中注入高斯噪声，以逼近真实世界的传感器缺陷。

5. **误差评估分析**: `trilat_eval` 节点抓取 UKF 估计值并与 Gazebo 引擎的绝对真实值 (Ground Truth) 进行对比，实时计算 RMSE 等核心指标。

该架构利用低频的 UWB 绝对距离更新，从物理层面上有效截断了高频里程计的累积漂移。

---

## 📈 预期实验结果

- 仅依赖里程计的轨迹随时间推移出现严重的结构性漂移。
- 接入 UWB + UKF 融合后，多机器人间的相对定位误差被大幅压缩并收敛。
- 算法框架在拓扑逻辑上支持向 3 至 N 台机器人集群平滑扩展。

---

## 🛠️ 未来规划与贡献方向

- 移植适配真实的 UWB 物理硬件 (如 Decawave, Qorvo, Pozyx 等)。
- 引入基于图优化的后端算法 (Pose Graph) 以保障全局一致性。
- 支持机器人节点的动态切入与剔除。
- 增加针对 RViz2 和 PlotJuggler 的可视化预设文件。
- 在物理移动平台 (如 TurtleBot, Jackal) 上完成真机验证。

---

## 📄 开源协议

本项目采用 **MIT License** 开源协议。详细信息请参阅 [LICENSE](LICENSE) 文件。

## 🤝 参与贡献
我们欢迎任何形式的贡献！请随时提交 **Issue** 探讨问题，或发起 **Pull Request** 提交代码。

- 项目维护者: [crzzo0129](https://github.com/crzzo0129)

如果您在复现过程中有任何疑问、改进建议，或是希望分享您的实验数据，请通过 Issue 与我们联系。

---

⭐ 如果本项目对您的科研或开发有帮助，请点亮 **Star**！

---
