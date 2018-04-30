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