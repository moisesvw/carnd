#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
  MatrixXd GenerateSigmas(const VectorXd &x, const MatrixXd &p, const int n_x, const double a, const double yaw, const double lambda_);
  MatrixXd PredictSigmas(const MatrixXd &state, const double delta_t, const int n_x);
  void MeanAndCovariance(const MatrixXd &predictions, const VectorXd &weights, const int n_a, VectorXd* x, MatrixXd* p);
  void GetRadarMeasurement(const VectorXd &state, VectorXd* x);
  void PredictRadarMeasurement(const MatrixXd &state,
                               const int n_z, const int n_a,
                               const VectorXd &weights, const MatrixXd R,
                               VectorXd* z, MatrixXd* s);
  void UpdateState(const VectorXd &z_state, const VectorXd &z_pred,
                  const MatrixXd &x_sig, const MatrixXd &z_sig, const MatrixXd &s,
                  const int n_x, const int n_z, const int n_a, const VectorXd &weights,
                  VectorXd* x_state, MatrixXd* p);

};

#endif /* TOOLS_H_ */