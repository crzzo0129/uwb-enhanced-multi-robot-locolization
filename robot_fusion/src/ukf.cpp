#include "robot_fusion/ukf.hpp"
#include <iostream>

namespace robot_fusion {

UKF::UKF(double dt)
    : dt_(dt), alpha_(1e-3), beta_(2.0), kappa_(0.0), num_sigma_points_(2 * STATE_DIM + 1) {
  lambda_ = alpha_ * alpha_ * (STATE_DIM + kappa_) - STATE_DIM;
  
  // Calculate weights
  double gamma = std::sqrt(STATE_DIM + lambda_);
  Wm_.resize(num_sigma_points_);
  Wc_.resize(num_sigma_points_);
  
  Wm_[0] = lambda_ / (STATE_DIM + lambda_);
  Wc_[0] = lambda_ / (STATE_DIM + lambda_) + (1 - beta_ + 2 * 2);  // alpha^2 = 1 for Gaussian
  
  for (int i = 1; i < num_sigma_points_; ++i) {
    Wm_[i] = 1.0 / (2.0 * (STATE_DIM + lambda_));
    Wc_[i] = 1.0 / (2.0 * (STATE_DIM + lambda_));
  }

  // Initialize state and covariance with proper sizes
  x_ = Eigen::VectorXd::Zero(STATE_DIM);
  P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  Q_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  R_odom_ = Eigen::MatrixXd::Identity(ODOM_MEAS_DIM, ODOM_MEAS_DIM);
}

void UKF::initialize(
    const Eigen::VectorXd& initial_state,
    const Eigen::MatrixXd& initial_p,
    const Eigen::MatrixXd& process_noise,
    const Eigen::MatrixXd& odom_noise) {
  x_ = initial_state;
  P_ = initial_p;
  Q_ = process_noise;
  R_odom_ = odom_noise;
}

Eigen::MatrixXd UKF::generateSigmaPoints(
    const Eigen::VectorXd& x,
    const Eigen::MatrixXd& P) {
  double gamma = std::sqrt(STATE_DIM + lambda_);
  Eigen::MatrixXd sigma_points(STATE_DIM, num_sigma_points_);
  
  // First sigma point is the mean
  sigma_points.col(0) = x;
  
  // Cholesky decomposition
  Eigen::LLT<Eigen::MatrixXd> llt(P);
  Eigen::MatrixXd L = llt.matrixL();
  
  // Positive and negative deviations
  Eigen::MatrixXd sqrt_term = gamma * L;
  for (int i = 0; i < STATE_DIM; ++i) {
    sigma_points.col(i + 1) = x + sqrt_term.col(i);
    sigma_points.col(STATE_DIM + i + 1) = x - sqrt_term.col(i);
  }
  
  return sigma_points;
}

double UKF::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double UKF::angleError(double expected, double measured) {
  double diff = measured - expected;
  return normalizeAngle(diff);
}

Eigen::VectorXd UKF::motionModel(
    const Eigen::VectorXd& state,
    double dt) {
  Eigen::VectorXd next_state = state;
  
  double x = state(0);
  double y = state(1);
  double theta = state(2);
  double vx = state(3);
  double vy = state(4);
  double omega = state(5);
  
  // Simple constant velocity model
  // Transform velocity from body frame to world frame
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);
  
  double v_world_x = vx * cos_theta - vy * sin_theta;
  double v_world_y = vx * sin_theta + vy * cos_theta;
  
  next_state(0) = x + v_world_x * dt;
  next_state(1) = y + v_world_y * dt;
  next_state(2) = normalizeAngle(theta + omega * dt);
  // Velocity and angular velocity remain constant (or can have small decay)
  next_state(3) = vx * 0.99;  // Small damping
  next_state(4) = vy * 0.99;
  next_state(5) = omega * 0.99;
  
  return next_state;
}

Eigen::Vector3d UKF::measurementModelOdom(
    const Eigen::VectorXd& state) {
  Eigen::Vector3d z;
  z(0) = state(0);  // x
  z(1) = state(1);  // y
  z(2) = state(2);  // theta
  return z;
}

Eigen::Vector2d UKF::measurementModelTrilat(
    const Eigen::VectorXd& state) {
  // This extracts the relative position as seen from current heading
  // In practice, this would be the position in the body frame
  double x = state(0);
  double y = state(1);
  
  // For now, just return global position - the actual relative position
  // would depend on which other robot we're measuring relative to
  Eigen::Vector2d z;
  z(0) = x;
  z(1) = y;
  return z;
}

Eigen::MatrixXd UKF::propagateSigmaPoints(
    const Eigen::MatrixXd& sigma_points,
    double dt) {
  Eigen::MatrixXd propagated(STATE_DIM, num_sigma_points_);
  for (int i = 0; i < num_sigma_points_; ++i) {
    propagated.col(i) = motionModel(sigma_points.col(i), dt);
  }
  return propagated;
}

void UKF::predict(double dt) {
  if (dt < 0) dt = dt_;
  
  // Generate sigma points
  Eigen::MatrixXd sigma_points = generateSigmaPoints(x_, P_);
  
  // Propagate through motion model
  Eigen::MatrixXd sigma_pred = propagateSigmaPoints(sigma_points, dt);
  
  // Calculate predicted mean
  Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(STATE_DIM);
  for (int i = 0; i < num_sigma_points_; ++i) {
    x_pred += Wm_[i] * sigma_pred.col(i);
  }
  x_pred(2) = normalizeAngle(x_pred(2));
  
  // Calculate predicted covariance
  Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  for (int i = 0; i < num_sigma_points_; ++i) {
    Eigen::VectorXd error = sigma_pred.col(i) - x_pred;
    error(2) = normalizeAngle(error(2));
    P_pred += Wc_[i] * error * error.transpose();
  }
  P_pred += Q_;
  
  // Update state and covariance
  x_ = x_pred;
  P_ = P_pred;
}

void UKF::updateOdom(const Eigen::Vector3d& z) {
  // Generate sigma points
  Eigen::MatrixXd sigma_points = generateSigmaPoints(x_, P_);
  
  // Transform sigma points to measurement space
  std::vector<Eigen::Vector3d> z_sigma(num_sigma_points_);
  for (int i = 0; i < num_sigma_points_; ++i) {
    z_sigma[i] = measurementModelOdom(sigma_points.col(i));
  }
  
  // Calculate predicted measurement mean
  Eigen::Vector3d z_pred = Eigen::Vector3d::Zero();
  for (int i = 0; i < num_sigma_points_; ++i) {
    z_pred += Wm_[i] * z_sigma[i];
  }
  z_pred(2) = normalizeAngle(z_pred(2));
  
  // Calculate innovation covariance Pz and cross-covariance Pxz
  Eigen::Matrix3d Pz = Eigen::Matrix3d::Zero();
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(STATE_DIM, 3);
  
  for (int i = 0; i < num_sigma_points_; ++i) {
    Eigen::Vector3d z_diff = z_sigma[i] - z_pred;
    z_diff(2) = angleError(z_pred(2), z_sigma[i](2));
    
    Eigen::VectorXd x_diff = sigma_points.col(i) - x_;
    x_diff(2) = normalizeAngle(x_diff(2));
    
    Pz += Wc_[i] * z_diff * z_diff.transpose();
    Pxz += Wc_[i] * x_diff * z_diff.transpose();
  }
  Pz += R_odom_;
  
  // Calculate Kalman gain
  Eigen::MatrixXd K = Pxz * Pz.inverse();
  
  // Calculate innovation
  Eigen::Vector3d y = z - z_pred;
  y(2) = angleError(z_pred(2), z(2));
  
  // Update state
  x_ += K * y;
  x_(2) = normalizeAngle(x_(2));
  
  // Update covariance using Joseph form for stability
  // P = (I - K*H)*P, where H is [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0]
  Eigen::MatrixXd I_minus_KH = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  I_minus_KH.block(0, 0, 3, STATE_DIM) -= K;
  P_ = I_minus_KH * P_;
}

void UKF::updateTrilatMeasurement(
    const Eigen::Vector2d& z_rel,
    const Eigen::Matrix<double, 2, 2>& meas_noise) {
  // Generate sigma points
  Eigen::MatrixXd sigma_points = generateSigmaPoints(x_, P_);
  
  // Transform sigma points to measurement space (relative position)
  std::vector<Eigen::Vector2d> z_sigma(num_sigma_points_);
  for (int i = 0; i < num_sigma_points_; ++i) {
    z_sigma[i] = measurementModelTrilat(sigma_points.col(i));
  }
  
  // Calculate predicted measurement mean
  Eigen::Vector2d z_pred = Eigen::Vector2d::Zero();
  for (int i = 0; i < num_sigma_points_; ++i) {
    z_pred += Wm_[i] * z_sigma[i];
  }
  
  // Calculate innovation covariance Pz and cross-covariance Pxz
  Eigen::Matrix2d Pz = Eigen::Matrix2d::Zero();
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(STATE_DIM, 2);
  
  for (int i = 0; i < num_sigma_points_; ++i) {
    Eigen::Vector2d z_diff = z_sigma[i] - z_pred;
    Eigen::VectorXd x_diff = sigma_points.col(i) - x_;
    x_diff(2) = normalizeAngle(x_diff(2));
    
    Pz += Wc_[i] * z_diff * z_diff.transpose();
    Pxz += Wc_[i] * x_diff * z_diff.transpose();
  }
  Pz += meas_noise;
  
  // Calculate Kalman gain
  Eigen::MatrixXd K = Pxz * Pz.inverse();
  
  // Calculate innovation
  Eigen::Vector2d y = z_rel - z_pred;
  
  // Update state
  Eigen::VectorXd K_times_y = K * y;
  x_ += K_times_y;
  x_(2) = normalizeAngle(x_(2));
  
  // Update covariance using Joseph form for stability
  // P = (I - K*H)*P, where H is [1 0 0 0 0 0; 0 1 0 0 0 0]
  Eigen::MatrixXd I_minus_KH = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  I_minus_KH.block(0, 0, 2, STATE_DIM) -= K;
  P_ = I_minus_KH * P_;
}

}  // namespace robot_fusion
