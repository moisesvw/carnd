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
  MatrixXd GenerateSigmas(const VectorXd &x, const MatrixXd &p, const int n_x);
  MatrixXd GenerateSigmas(const VectorXd &x, const MatrixXd &p, const int n_x, const double a, const double yaw);
  MatrixXd PredictSigmas(const MatrixXd &state, const double delta_t);
  void MeanAndCovariance(const MatrixXd &predictions, VectorXd* x, MatrixXd* p);
  void GetRadarMeasurement(const VectorXd &state, VectorXd* x);

};

#endif /* TOOLS_H_ */