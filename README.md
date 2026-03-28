# uwb-enhanced-multi-robot-locolization
multi_robot_trilat:三角定位，每个机器人一个节点，输出话题/rbX/tri_pos_in_rbY以及/rbX/tri_pos_in_rbZ
odom_noise:在gazebo的odom基础上增加噪声，模拟真实情况
robot_fusion:
  gazebo仿真环境设置、
  uwb模拟插件（话题:/robot_distances）,一帧数据中xyz分别是12 23 13 的距离
  ukf节点，三个，对应三个机器人。读取odom和另外两个机器人坐标系下的坐标
trilat_eval:需要手动让机器人走一个圆形，定时采样机器人的坐标和ukf融合后坐标，对比效果
