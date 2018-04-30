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
  cout << sigmas << endl;
  return sigmas;
}