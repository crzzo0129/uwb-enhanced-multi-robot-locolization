# UWB-Enhanced Multi-Robot Localization

**UWB-Enhanced Multi-Robot Cooperative Localization System**  
*(ROS 2 + Gazebo Simulation)*

This project implements a framework for improving **relative localization** among multiple robots using **Ultra-Wideband (UWB)** ranging measurements. By fusing each robot's wheel odometry with inter-robot UWB distance observations, the system significantly reduces localization drift in GNSS-denied environments (e.g., indoors, underground, or dense forests).

Key techniques include:
- Multi-robot trilateration for relative position estimation
- Unscented Kalman Filter (UKF) for sensor fusion
- Realistic Gazebo simulation with a custom UWB ranging plugin

---

## ✨ Features

- **Infrastructure-free relative localization**: Robots estimate each other's positions in their local coordinate frames without external anchors or global references.
- **Robust sensor fusion**: Combines noisy odometry with UWB range measurements using UKF.
- **Modular architecture**:
  - `multi_robot_trilat/`: Multi-robot trilateration node
  - `robot_fusion/`: UKF fusion, Gazebo simulation, and UWB distance publisher
  - `odom_noise/`: Realistic odometry noise simulation
  - `trilat_eval/`: Localization accuracy evaluation tools (compared against ground truth)
- **Simulation-ready**: Supports 3+ robots with manual circular motion for evaluation.
- **ROS 2 native**: Uses standard message interfaces for easy extension to real hardware.

---

## 📁 Repository Structure

```
uwb-enhanced-multi-robot-locolization/
├── multi_robot_trilat/          # Multi-robot trilateration module
├── odom_noise/                  # Odometry noise simulation
├── robot_fusion/                # UKF fusion + Gazebo + UWB plugin
│   ├── launch/                  # Launch files
│   └── ...
├── trilat_eval/                 # Evaluation and metrics tools
├── uwbpsr_ratros2/              # UWB-related ROS 2 package (if present)
├── LICENSE
├── README.md
└── 指令.txt                     # Original Chinese running instructions
```

---

## 🚀 Quick Start

### Prerequisites

- **OS**: Ubuntu 20.04 or 22.04
- **ROS 2**: Humble (recommended) or Iron
- **Gazebo**: Harmonic or Classic (depending on ROS 2 version)
- **Build tools**: `colcon`, `rosdep`

### Installation

```bash
# 1. Create workspace
mkdir -p ~/uwb_ws/src
cd ~/uwb_ws/src

# 2. Clone the repository
git clone https://github.com/crzzo0129/uwb-enhanced-multi-robot-locolization.git

# 3. Install dependencies
cd ~/uwb_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
colcon build --symlink-install
source install/setup.bash
```

### Running the Simulation

Refer to the original `指令.txt` for detailed commands. Typical workflow:

```bash
# Launch the full multi-robot simulation with UWB and UKF (example)
ros2 launch robot_fusion multi_robot_ukf.launch.py

# Or launch individual components
ros2 run odom_noise odom_noise_node
ros2 run robot_fusion uwb_distance_publisher
ros2 run robot_fusion ukf_fusion_node --ros-args -r __ns:=/rb1
```

**Tip**: Use the `trilat_eval` module to collect data while manually driving robots in circular trajectories for accuracy assessment.

---

## 📊 System Overview

1. **UWB Ranging Simulation**: A Gazebo plugin publishes inter-robot distances (e.g., `/robot_distances` topic) with realistic noise.
2. **Trilateration**: The `multi_robot_trilat` package computes coarse relative positions using UWB ranges (outputs topics like `/rbX/tri_pos_in_rbY`).
3. **UKF Fusion**: Each robot runs an independent UKF node that fuses its own odometry with trilateration observations for smoother, more accurate pose estimates.
4. **Noise Modeling**: `odom_noise` adds Gaussian noise to Gazebo odometry to simulate real-world sensor imperfections.
5. **Evaluation**: `trilat_eval` compares UKF estimates against Gazebo ground truth and computes metrics such as RMSE.

This pipeline effectively mitigates odometry drift using occasional UWB range updates.

---

## 📈 Expected Results

- Pure odometry drifts significantly over time.
- With UWB + UKF fusion, relative positioning error is greatly reduced.
- The framework is extensible to 3–N robots.

---

## 🛠️ Future Work & Contributions

- Port to real UWB hardware (e.g., Decawave, Qorvo, Pozyx).
- Integrate graph-based optimization (Pose Graph) for global consistency.
- Support dynamic addition/removal of robots.
- Add visualization presets for RViz2 and PlotJuggler.
- Validate on physical platforms (TurtleBot, Jackal, etc.).

---

## 📄 License

This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for details.

---

## 🤝 Contributing

Contributions are welcome! Feel free to open an **Issue** or submit a **Pull Request**.

- Project maintainer: [crzzo0129](https://github.com/crzzo0129)

If you have questions, suggestions, or want to share experimental results, please let us know.

---

⭐ **Star this repository** if you find it helpful!

---
