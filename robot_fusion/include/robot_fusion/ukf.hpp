#ifndef ROBOT_FUSION_UKF_HPP_
#define ROBOT_FUSION_UKF_HPP_

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace robot_fusion {

/**
 * @class UKF
 * @brief Unscented Kalman Filter for 2D robot localization
 * 
 * State vector: [x, y, theta, vx, vy, omega]
 * Measurements: odom (x, y, theta) and trilateration relative poses
 */
class UKF {
public:
  static constexpr int STATE_DIM = 6;  // [x, y, theta, vx, vy, omega]
  static constexpr int ODOM_MEAS_DIM = 3;  // [x, y, theta]
  static constexpr int TRILAT_MEAS_DIM = 2;  // [x_rel, y_rel]

  /**
   * @brief Constructor
   * @param dt Time step between updates
   */
  explicit UKF(double dt = 0.05);

  /**
   * @brief Initialize filter state and covariance
   * @param initial_state Initial state [x, y, theta, vx, vy, omega]
   * @param initial_p Initial covariance matrix
   * @param process_noise Process noise covariance matrix
   * @param odom_noise Odometry measurement noise covariance
   */
  void initialize(
    const Eigen::VectorXd& initial_state,
    const Eigen::MatrixXd& initial_p,
    const Eigen::MatrixXd& process_noise,
    const Eigen::MatrixXd& odom_noise);

  /**
   * @brief Prediction step using constant velocity motion model
   * @param dt Time delta (if different from initialized dt)
   */
  void predict(double dt = -1.0);

  /**
   * @brief Update with odometry measurement
   * @param z Measurement [x, y, theta]
   */
  void updateOdom(const Eigen::Vector3d& z);

  /**
   * @brief Update with trilateration relative pose measurement
   * @param z_rel Relative pose in local frame [x_rel, y_rel]
   * @param meas_noise Measurement noise covariance
   */
  void updateTrilatMeasurement(
    const Eigen::Vector2d& z_rel,
    const Eigen::Matrix<double, TRILAT_MEAS_DIM, TRILAT_MEAS_DIM>& meas_noise);

  /**
   * @brief Get current state
   */
  const Eigen::VectorXd& getState() const { return x_; }

  /**
   * @brief Get current covariance
   */
  const Eigen::MatrixXd& getCovariance() const { return P_; }

  /**
   * @brief Get position estimate
   */
  Eigen::Vector2d getPosition() const { return x_.head<2>(); }

  /**
   * @brief Get yaw angle
   */
  double getYaw() const { return x_(2); }

  /**
   * @brief Get velocity
   */
  Eigen::Vector2d getVelocity() const { return x_.segment<2>(3); }

private:
  // UKF parameters
  double alpha_;  // Spread of sigma points
  double beta_;   // Distribution info (2.0 for Gaussian)
  double kappa_;  // Secondary scaling parameter
  double lambda_; // Scaling parameter

  double dt_;     // Time step

  // State and covariance
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  
  // Noise covariances
  Eigen::MatrixXd Q_;  // Process noise
  Eigen::MatrixXd R_odom_;  // Odom measurement noise

  // Sigma point weights
  std::vector<double> Wm_;  // Weights for mean
  std::vector<double> Wc_;  // Weights for covariance
  int num_sigma_points_;

  /**
   * @brief Generate sigma points
   * @param x Mean state
   * @param P Covariance matrix
   * @return Matrix with sigma points as columns
   */
  Eigen::MatrixXd generateSigmaPoints(
    const Eigen::VectorXd& x,
    const Eigen::MatrixXd& P);

  /**
   * @brief Process sigma points through motion model
   * @param sigma_points Input sigma points
   * @param dt Time step
   * @return Propagated sigma points
   */
  Eigen::MatrixXd propagateSigmaPoints(
    const Eigen::MatrixXd& sigma_points,
    double dt);

  /**
   * @brief Motion model: propagate state
   */
  Eigen::VectorXd motionModel(
    const Eigen::VectorXd& state,
    double dt);

  /**
   * @brief Measurement model for odometry
   */
  Eigen::Vector3d measurementModelOdom(
    const Eigen::VectorXd& state);

  /**
   * @brief Measurement model for trilateration
   * Extracts relative position from global position using heading
   */
  Eigen::Vector2d measurementModelTrilat(
    const Eigen::VectorXd& state);

  /**
   * @brief Normalize angle to [-pi, pi]
   */
  static double normalizeAngle(double angle);

  /**
   * @brief Angle difference handling
   */
  static double angleError(double expected, double measured);
};

}  // namespace robot_fusion

#endif  // ROBOT_FUSION_UKF_HPP_
