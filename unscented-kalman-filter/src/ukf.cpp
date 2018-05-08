#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
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
  std_a_ = 7.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 7.0;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;
  n_a = n_aug_ * 2 + 1;
  // Initializing weights
  weights_ = VectorXd(n_a);
  weights_(0) = lambda_/(lambda_ + n_aug_);
  for(int i=1; i < n_a; i++){
    weights_(i) = 0.5/(lambda_ + n_aug_);
  }
  // P
  P_ = MatrixXd::Identity(n_x_, n_x_);
  x_ = VectorXd::Zero(n_x_);
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      float ro = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float ro_dot = meas_package.raw_measurements_[2];

      double px = ro * cos(phi);
      double py = ro * sin(phi);
      double vx = ro_dot * cos(phi);
      double vy = ro_dot * sin(phi);
      double v  = sqrt(vx * vx + vy * vy);
      x_ << px, py, v, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      double px = meas_package.raw_measurements_[0];
      double py = meas_package.raw_measurements_[1];
      x_ << px, py, 0, 0, 0;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt expressed in seconds
  Prediction(dt);

  if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
    UpdateRadar(meas_package);
  }

  if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  MatrixXd sigmas = tools.GenerateSigmas(x_, P_, n_aug_, std_a_, std_yawdd_);
  Xsig_pred_ = tools.PredictSigmas(sigmas, delta_t);
  tools.MeanAndCovariance(Xsig_pred_, &x_, &P_);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  Also calculates the lidar NIS.
  */

  int n_z = 2;
  MatrixXd Zsig = Xsig_pred_.topRows(n_z);

  VectorXd z_ = VectorXd(n_z);
  VectorXd z = meas_package.raw_measurements_;
  MatrixXd s_ = MatrixXd(n_z, n_z);

  z_.fill(0.0);
  for(int i=0; i < n_a; i++){
    z_ = z_ + weights_(i) * Zsig.col(i);
  }

  MatrixXd R = MatrixXd::Zero(n_z, n_z);
  R(0,0) = std_laspx_ * std_laspx_;
  R(1,1) = std_laspy_ * std_laspy_;

  s_.fill(0.0);
  for(int i=0; i < n_a; i++){
    VectorXd z_diff = Zsig.col(i) - z_;
    s_ = s_ + weights_(i) * z_diff * z_diff.transpose();
  }
  s_ = s_ + R;

  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for(int i=0; i < n_a; i++){
    VectorXd z_diff = Zsig.col(i) - z_;
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd gain = Tc * s_.inverse();
  VectorXd z_diff = z - z_;

  x_ = x_ + gain * z_diff;
  P_ = P_ - gain * s_ * gain.transpose();
  double NIS = z_diff.transpose() * s_.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  Also calculates the radar NIS.
  */

  int n_z = 3;
  VectorXd z_ = VectorXd(n_z);
  VectorXd z  = meas_package.raw_measurements_;
  MatrixXd s_ = MatrixXd(n_z, n_z);
  tools.PredictRadarMeasurement(Xsig_pred_, &z_, &s_);
  MatrixXd Zsig_pred_ = MatrixXd(n_z, n_a);
  VectorXd x__ = VectorXd(n_z);

  for(int i=0; i < n_a; i++){
    tools.GetRadarMeasurement(Xsig_pred_.col(i), &x__);
    Zsig_pred_.col(i) = x__;
  }

  tools.UpdateState(z, z_, Xsig_pred_, Zsig_pred_, s_, &x_, &P_);
}
