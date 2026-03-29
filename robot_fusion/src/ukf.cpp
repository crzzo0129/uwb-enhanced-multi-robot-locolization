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
  Wc_[0] = lambda_ / (STATE_DIM + lambda_) + (1 - beta_ + 2 * 2);

  for (int i = 1; i < num_sigma_points_; ++i) {
    Wm_[i] = 1.0 / (2.0 * (STATE_DIM + lambda_));
    Wc_[i] = 1.0 / (2.0 * (STATE_DIM + lambda_));
  }

  // Initialize state and covariance
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
  
  sigma_points.col(0) = x;
  
  Eigen::LLT<Eigen::MatrixXd> llt(P);
  Eigen::MatrixXd L = llt.matrixL();
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
  
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);
  
  double v_world_x = vx * cos_theta - vy * sin_theta;
  double v_world_y = vx * sin_theta + vy * cos_theta;
  
  next_state(0) = x + v_world_x * dt;
  next_state(1) = y + v_world_y * dt;
  next_state(2) = normalizeAngle(theta + omega * dt);
  next_state(3) = vx * 0.99;
  next_state(4) = vy * 0.99;
  next_state(5) = omega * 0.99;
  
  return next_state;
}

Eigen::Vector3d UKF::measurementModelOdom(
    const Eigen::VectorXd& state) {
  Eigen::Vector3d z;
  z(0) = state(0);
  z(1) = state(1);
  z(2) = state(2);
  return z;
}

Eigen::VectorXd UKF::measurementModelTrilat(
    const Eigen::VectorXd& state, const Eigen::Vector2d& other_pos) {
  Eigen::Vector2d my_pos = state.head<2>();
  double predicted_range = (my_pos - other_pos).norm();
  Eigen::VectorXd z(1);
  z(0) = predicted_range;
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
  
  Eigen::MatrixXd sigma_points = generateSigmaPoints(x_, P_);
  Eigen::MatrixXd sigma_pred = propagateSigmaPoints(sigma_points, dt);
  
  // Predicted mean
  Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(STATE_DIM);
  for (int i = 0; i < num_sigma_points_; ++i) {
    x_pred += Wm_[i] * sigma_pred.col(i);
  }
  x_pred(2) = normalizeAngle(x_pred(2));
  
  // Predicted covariance
  Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  for (int i = 0; i < num_sigma_points_; ++i) {
    Eigen::VectorXd error = sigma_pred.col(i) - x_pred;
    error(2) = normalizeAngle(error(2));
    P_pred += Wc_[i] * error * error.transpose();
  }
  P_pred += Q_;
  
  x_ = x_pred;
  P_ = P_pred;
}

// ==================== 修复后的 updateOdom ====================
void UKF::updateOdom(const Eigen::Vector3d& z) {
  Eigen::MatrixXd sigma_points = generateSigmaPoints(x_, P_);
  
  std::vector<Eigen::Vector3d> z_sigma(num_sigma_points_);
  for (int i = 0; i < num_sigma_points_; ++i) {
    z_sigma[i] = measurementModelOdom(sigma_points.col(i));
  }
  
  Eigen::Vector3d z_pred = Eigen::Vector3d::Zero();
  for (int i = 0; i < num_sigma_points_; ++i) {
    z_pred += Wm_[i] * z_sigma[i];
  }
  z_pred(2) = normalizeAngle(z_pred(2));
  
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
  
  Eigen::MatrixXd K = Pxz * Pz.inverse();
  
  Eigen::Vector3d y = z - z_pred;
  y(2) = angleError(z_pred(2), z(2));
  
  // State update
  x_ += K * y;
  x_(2) = normalizeAngle(x_(2));
  
  // Joseph form（更稳定）
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  I.block(0, 0, STATE_DIM, 3) -= K;
  P_ = (I * P_ * I.transpose()) + (K * R_odom_ * K.transpose());
}

void UKF::updateTrilatMeasurement(
    double measured_range, const Eigen::Vector2d& other_pos,
    double range_noise_std) {

  Eigen::MatrixXd sigma_points = generateSigmaPoints(x_, P_);

  // Predict range for each sigma point
  std::vector<double> z_sigma(num_sigma_points_);
  for (int i = 0; i < num_sigma_points_; ++i) {
    z_sigma[i] = measurementModelTrilat(sigma_points.col(i), other_pos)(0);
  }

  // Predicted measurement mean
  double z_pred = 0.0;
  for (int i = 0; i < num_sigma_points_; ++i) {
    z_pred += Wm_[i] * z_sigma[i];
  }

  // Innovation covariance and cross-covariance
  double Pz = 0.0;
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(STATE_DIM, 1);

  for (int i = 0; i < num_sigma_points_; ++i) {
    double z_diff = z_sigma[i] - z_pred;
    Eigen::VectorXd x_diff = sigma_points.col(i) - x_;
    x_diff(2) = normalizeAngle(x_diff(2));

    Pz += Wc_[i] * z_diff * z_diff;
    Pxz += Wc_[i] * x_diff * z_diff;
  }

  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1) * (range_noise_std * range_noise_std);
  Pz += R(0, 0);

  // Kalman gain
  Eigen::MatrixXd K = Pxz * (1.0 / Pz);

  // Innovation
  double y = measured_range - z_pred;

  // State update
  x_ += K * y;
  x_(2) = normalizeAngle(x_(2));

  // Joseph form covariance update (稳定)
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  I.block(0, 0, STATE_DIM, 1) -= K;
  P_ = (I * P_ * I.transpose()) + (K * R * K.transpose());
}

}  // namespace robot_fusion