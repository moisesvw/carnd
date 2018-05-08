#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  if(estimations.size() == 0 || ground_truth.size() == 0){
    return rmse ;
  }

  if(estimations.size() != ground_truth.size()){
    return rmse ;
  }

  int n = estimations.size();
  for(int i=0; i < n; ++i){
    rmse = rmse.array() + (estimations[i] - ground_truth[i]).array().square();
  }
  rmse = (rmse/n).array().sqrt();
  return rmse;
}

MatrixXd Tools::GenerateSigmas(const VectorXd &x, const MatrixXd &p, const int n_x, const double a, const double yaw, const double lambda_){
  VectorXd x_aug(n_x);
  MatrixXd p_aug(n_x, n_x);
  p_aug =  MatrixXd::Zero(n_x, n_x);
  x_aug = VectorXd::Zero(n_x);
  x_aug.head(5) = x;
  p_aug.topLeftCorner(5, 5) = p;
  p_aug(n_x-2, n_x -2) = a*a;
  p_aug(n_x-1, n_x -1) = yaw*yaw;

  MatrixXd sigmas = MatrixXd(n_x, 2 * n_x +1);
  MatrixXd A = p_aug.llt().matrixL();
  sigmas.col(0) = x_aug;
  for(int i=0; i<n_x; i++){
    sigmas.col(i+1) =  x_aug + sqrt(lambda_+n_x) * A.col(i);
    sigmas.col(i+1+n_x) = x_aug - sqrt(lambda_+n_x) * A.col(i);
  }

  return sigmas;
}

MatrixXd Tools::PredictSigmas(const MatrixXd &state, const double delta_t, const int n_x){
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

    if(fabs(yaw_rate_k) > 0.001){
      x_delta <<
        v_k/yaw_rate_k * (sin(yaw_k + yaw_rate_k*delta_t) - sin(yaw_k)),
        v_k/yaw_rate_k * (-cos(yaw_k + yaw_rate_k*delta_t) + cos(yaw_k)),
        0,
        yaw_rate_k*delta_t,
        0;
    } else {
      x_delta << v_k * cos(yaw_k) * delta_t, v_k * sin(yaw_k) * delta_t, 0, 0, 0;
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

void Tools::MeanAndCovariance(const MatrixXd &predictions, const VectorXd &weights, const int n_a, VectorXd* x, MatrixXd* p){
  VectorXd x_ = *x;
  MatrixXd p_ = *p;
  p_.fill(0.0);
  x_.fill(0.0);

  for(int i=0; i < n_a; i++){
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

void Tools::PredictRadarMeasurement(const MatrixXd &state, const int n_z, const int n_a,
                                    const double std_radr, const double std_radphi, const double std_radrd,
                                    VectorXd &weights,
                                    VectorXd* z, MatrixXd* s){

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

void Tools::UpdateState(const VectorXd &z_state, const VectorXd &z_pred,
                        const MatrixXd &x_sig, const MatrixXd &z_sig, const MatrixXd s,
                        const int n_x, const int n_z, const int n_a, VectorXd &weights,
                        VectorXd* x_state, MatrixXd* p){

  MatrixXd Tc = MatrixXd(n_x, n_z);
  Tc.fill(0.0);
  for(int i=0; i < n_a; i++){
    VectorXd z_diff = z_sig.col(i) - z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    VectorXd x_diff = x_sig.col(i) - *x_state;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  MatrixXd gain = Tc * s.inverse();
  VectorXd z_diff = z_state - z_pred;
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  *x_state = *x_state + gain * z_diff;
  *p = *p - gain * s * gain.transpose();
}