#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}

MatrixXd Tools::GenerateSigmas(const VectorXd &x, const MatrixXd &p, const int n_x){
  float lambda_ = 3 - n_x;
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x +1);
  MatrixXd A = p.llt().matrixL();
  Xsig.col(0) = x;
  for(int i=0; i<n_x; i++){
    Xsig.col(i+1) =  x + sqrt(lambda_+n_x) * A.col(i);
    Xsig.col(i+1+n_x) =  x - sqrt(lambda_+n_x) * A.col(i);
  }

  return Xsig;
}

MatrixXd Tools::GenerateSigmas(const VectorXd &x, const MatrixXd &p, const int n_x, const double a, const double yaw){
  VectorXd x_aug(n_x);
  MatrixXd p_aug(n_x, n_x);
  p_aug =  MatrixXd::Zero(n_x, n_x);
  x_aug = VectorXd::Zero(n_x);
  x_aug.head(5) = x;
  p_aug.topLeftCorner(5, 5) = p;
  p_aug(n_x-2, n_x -2) = a*a;
  p_aug(n_x-1, n_x -1) = yaw*yaw;
  MatrixXd sigmas = GenerateSigmas(x_aug, p_aug, n_x);
  return sigmas;
}

MatrixXd Tools::PredictSigmas(const MatrixXd &state, const double delta_t){
  int n_x  = 5;
  int n_aug_cols = state.cols();
  MatrixXd prediction(n_x, n_aug_cols);
  VectorXd  x_k(n_x);
  VectorXd  x_delta(n_x);
  VectorXd  noise_delta(n_x);
  prediction = MatrixXd::Zero(n_x, n_aug_cols);
  double v_k, yaw_k, yaw_rate_k;
  double noise_a_k, noise_yaw_rate_k;

  for(int i = 0; i < n_aug_cols; i++){
    x_k = state.col(i).topRows(n_x);
    v_k = x_k(2); yaw_k = x_k(3) ; yaw_rate_k = x_k(4);

    if(yaw_rate_k == 0){
      x_delta << v_k * cos(yaw_k) * delta_t, v_k * sin(yaw_k) * delta_t, 0, 0, 0;
    } else {
      x_delta <<
        v_k/yaw_rate_k * (sin(yaw_k + yaw_rate_k*delta_t) - sin(yaw_k)),
        v_k/yaw_rate_k * (-cos(yaw_k + yaw_rate_k*delta_t) + cos(yaw_k)),
        0,
        yaw_rate_k*delta_t,
        0;
    }

    noise_a_k = state.col(i)(5);
    noise_yaw_rate_k = state.col(i)(6);
    noise_delta <<
      0.5 * (delta_t*delta_t) * cos(yaw_k) * noise_a_k,
      0.5 * (delta_t*delta_t) * sin(yaw_k) * noise_a_k,
      delta_t * noise_a_k,
      0.5 * (delta_t*delta_t) * noise_yaw_rate_k,
      delta_t * noise_yaw_rate_k; 

    prediction.col(i) = x_k + x_delta + noise_delta;
  }

  return prediction;
}

void Tools::MeanAndCovariance(const MatrixXd &predictions, VectorXd* x, MatrixXd* p){
  int n_x = 5;
  int n_aug = 7;
  int n_a = n_aug * 2 + 1;
  double lambda = 3 - n_aug;
  VectorXd x_(n_x);
  MatrixXd p_(n_x, n_x);
  VectorXd weights(n_a);
  p_.fill(0.0);
  x_.fill(0.0);
  weights(0) = lambda/(lambda + n_aug);
  for(int i=1; i < n_a; i++){
    weights(i) = 0.5/(lambda + n_aug);
  }

  for(int i=0; i < n_aug * 2 + 1; i++){
    x_ = x_ + weights(i) * predictions.col(i);
  }

  for(int i=0; i < n_a; i++){
    VectorXd x_diff = predictions.col(i) - x_;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    p_ = p_ + weights(i) * x_diff * x_diff.transpose();
  }

  *x = x_;
  *p = p_;
}

void Tools::GetRadarMeasurement(const VectorXd &state, VectorXd* x){
  //state is a vector [px, py, v, yaw, yaw_rate]
  VectorXd x_(3);
  double p = sqrt(state(0) * state(0) + state(1) * state(1));
  double a = atan(state(1)/state(0));
  double ro = (state(0) * cos(state(3)) * state(2) +  state(1) * sin(state(3)) * state(2))/p;
  x_ << p, a, ro;
  *x = x_;
}

void Tools::PredictRadarMeasurement(const MatrixXd &state, VectorXd* z, MatrixXd* s){
  int n_aug = 7;
  int n_a = n_aug * 2 + 1;
  int n_z = 3;
  double lambda = 3 - n_aug;
  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;
  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;
  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  VectorXd weights = VectorXd(n_a);
  weights(0) = lambda/(lambda + n_aug);
  for(int i=1; i < n_a; i++){
    weights(i) = 0.5/(lambda + n_aug);
  }

  MatrixXd Zsig = MatrixXd(n_z, n_a);
  VectorXd z_ = VectorXd(n_z);
  MatrixXd s_ = MatrixXd(n_z, n_z);
  VectorXd x_ = VectorXd(n_z);

  for(int i=0; i < n_a; i++){
    GetRadarMeasurement(state.col(i), &x_);
    Zsig.col(i) = x_;
  }

  z_.fill(0.0);
  for(int i=0; i < n_a; i++){
    z_ = z_ + weights(i) * Zsig.col(i);
  }

  MatrixXd R = MatrixXd::Zero(n_z, n_z);
  R(0,0) = std_radr * std_radr;
  R(1,1) = std_radphi * std_radphi;
  R(2,2) = std_radrd * std_radrd;
  s_.fill(0.0);
    for(int i=0; i < n_a; i++){
    VectorXd z_diff = Zsig.col(i) - z_;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    s_ = s_ + weights(i) * z_diff * z_diff.transpose();
  }
  s_ = s_ + R;
  *z = z_;
  *s = s_;
}