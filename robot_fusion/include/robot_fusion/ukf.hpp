#ifndef ROBOT_FUSION_UKF_HPP_
#define ROBOT_FUSION_UKF_HPP_

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace robot_fusion {

/**
 * @class UKF
 * @brief Unscented Kalman Filter for 2D robot localization (Range-only version)
 * 
 * State vector: [x, y, theta, vx, vy, omega]
 * Measurements: odom (x, y, theta) + trilateration range (distance only)
 */
class UKF {
public:
  static constexpr int STATE_DIM = 6;  // [x, y, theta, vx, vy, omega]
  static constexpr int ODOM_MEAS_DIM = 3;  // [x, y, theta]
  static constexpr int TRILAT_MEAS_DIM = 1;  // 现在只用距离（标量）

  explicit UKF(double dt = 0.05);

  void initialize(
    const Eigen::VectorXd& initial_state,
    const Eigen::MatrixXd& initial_p,
    const Eigen::MatrixXd& process_noise,
    const Eigen::MatrixXd& odom_noise);

  void predict(double dt = -1.0);

  void updateOdom(const Eigen::Vector3d& z);

  /**
   * @brief Range-only trilateration update
   * @param measured_range 实际测量的两机器人距离
   * @param other_pos       对方机器人的当前估计位置 [x, y]
   * @param range_noise_std 距离噪声标准差（来自 yaml 中的 trilat_noise）
   */
  void updateTrilatMeasurement(
    double measured_range,
    const Eigen::Vector2d& other_pos,
    double range_noise_std);

  const Eigen::VectorXd& getState() const { return x_; }
  const Eigen::MatrixXd& getCovariance() const { return P_; }

  Eigen::Vector2d getPosition() const { return x_.head<2>(); }
  double getYaw() const { return x_(2); }
  Eigen::Vector2d getVelocity() const { return x_.segment<2>(3); }

private:
  double alpha_, beta_, kappa_, lambda_, dt_;

  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_odom_;

  std::vector<double> Wm_, Wc_;
  int num_sigma_points_;

  Eigen::MatrixXd generateSigmaPoints(
    const Eigen::VectorXd& x,
    const Eigen::MatrixXd& P);

  Eigen::MatrixXd propagateSigmaPoints(
    const Eigen::MatrixXd& sigma_points,
    double dt);

  Eigen::VectorXd motionModel(
    const Eigen::VectorXd& state,
    double dt);

  Eigen::Vector3d measurementModelOdom(
    const Eigen::VectorXd& state);

  /**
   * @brief Range-only measurement model
   * @param state     本机状态
   * @param other_pos 对方位置 [x, y]
   * @return 预测的距离（标量，包装成 VectorXd）
   */
  Eigen::VectorXd measurementModelTrilat(
    const Eigen::VectorXd& state,
    const Eigen::Vector2d& other_pos);

  static double normalizeAngle(double angle);
  static double angleError(double expected, double measured);
};

}  // namespace robot_fusion

#endif  // ROBOT_FUSION_UKF_HPP_