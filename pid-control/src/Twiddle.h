#ifndef TWIDDLE_H
#define TWIDDLE_H
#include<random>
using namespace std;

class Twiddle {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  bool initialized;
  double prev_cte=-1;
  double sum_cte;
  double dt_cte;
  double cte;

  /**
   * 
   *  Twiddle state parameters
   */

  double p[3];
  double dp[3];
  double tol;
  double best_err;
  double err;
  int it = 0;
  int i = 0;
  bool flag_1;
  bool flag_2;
  bool flag_3;

  /*
  * Constructor
  */
  Twiddle();

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  /*
  * Initialize PID.
  */
  void Init();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * next angle steering
  */
  double Train(double cte);

};

#endif /* TWIDDLE_H */
