#include "ukf.h"
#include "Eigen/Dense"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace {
constexpr double kEpsilon = 1e-6;

void NormalizeAngle(double &angle) {
  while (angle > M_PI) angle -= 2. * M_PI;
  while (angle < -M_PI) angle += 2. * M_PI;
}
}  // namespace

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  is_initialized_ = false;
  time_us_ = 0;

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  x_.fill(0.0);
  P_ = MatrixXd::Identity(n_x_, n_x_);
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < weights_.size(); ++i) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    x_.fill(0.0);
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    } else {
      const double rho = meas_package.raw_measurements_(0);
      const double phi = meas_package.raw_measurements_(1);
      const double rho_dot = meas_package.raw_measurements_(2);
      x_(0) = rho * std::cos(phi);
      x_(1) = rho * std::sin(phi);
      x_(2) = std::fabs(rho_dot);
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  const double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0.0;
  x_aug(n_x_ + 1) = 0.0;

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  MatrixXd L = P_aug.llt().matrixL();
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  const double sigma_scale = std::sqrt(lambda_ + n_aug_);
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i + 1) = x_aug + sigma_scale * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sigma_scale * L.col(i);
  }

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    const double px = Xsig_aug(0, i);
    const double py = Xsig_aug(1, i);
    const double v = Xsig_aug(2, i);
    const double yaw = Xsig_aug(3, i);
    const double yawd = Xsig_aug(4, i);
    const double nu_a = Xsig_aug(5, i);
    const double nu_yawdd = Xsig_aug(6, i);

    double px_p;
    double py_p;
    if (std::fabs(yawd) > kEpsilon) {
      px_p = px + v / yawd * (std::sin(yaw + yawd * delta_t) - std::sin(yaw));
      py_p = py + v / yawd * (-std::cos(yaw + yawd * delta_t) + std::cos(yaw));
    } else {
      px_p = px + v * delta_t * std::cos(yaw);
      py_p = py + v * delta_t * std::sin(yaw);
    }

    Xsig_pred_(0, i) = px_p + 0.5 * nu_a * delta_t * delta_t * std::cos(yaw);
    Xsig_pred_(1, i) = py_p + 0.5 * nu_a * delta_t * delta_t * std::sin(yaw);
    Xsig_pred_(2, i) = v + nu_a * delta_t;
    Xsig_pred_(3, i) = yaw + yawd * delta_t + 0.5 * nu_yawdd * delta_t * delta_t;
    Xsig_pred_(4, i) = yawd + nu_yawdd * delta_t;
  }

  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd diff = Xsig_pred_.col(i) - x_;
    NormalizeAngle(diff(3));
    P_ += weights_(i) * diff * diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  const int n_z = 2;
  MatrixXd H = MatrixXd(n_z, n_x_);
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;

  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;

  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_pred = H * x_;
  VectorXd y = z - z_pred;
  MatrixXd S = H * P_ * H.transpose() + R;
  MatrixXd K = P_ * H.transpose() * S.inverse();

  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  P_ = (I - K * H) * P_;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  const int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    const double px = Xsig_pred_(0, i);
    const double py = Xsig_pred_(1, i);
    const double v = Xsig_pred_(2, i);
    const double yaw = Xsig_pred_(3, i);
    const double vx = std::cos(yaw) * v;
    const double vy = std::sin(yaw) * v;
    const double rho = std::max(std::sqrt(px * px + py * py), kEpsilon);

    Zsig(0, i) = rho;
    Zsig(1, i) = std::atan2(py, px);
    Zsig(2, i) = (px * vx + py * vy) / rho;
  }

  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd diff = Zsig.col(i) - z_pred;
    NormalizeAngle(diff(1));
    S += weights_(i) * diff * diff.transpose();
  }
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;
  S += R;

  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    NormalizeAngle(x_diff(3));
    VectorXd z_diff = Zsig.col(i) - z_pred;
    NormalizeAngle(z_diff(1));
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  NormalizeAngle(z_diff(1));
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}
