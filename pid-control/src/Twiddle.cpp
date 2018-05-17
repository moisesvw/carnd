#include "Twiddle.h"
#include <random>
#include <iostream>

using namespace std;

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init() {
  if(!initialized){
    p[0] = 0.0;
    p[1] = 0.0;
    p[2] = 0.0;
    dp[0] = 0.01;
    dp[1] = 0.01;
    dp[2] = 0.01;
    initialized = true;
    flag_1 = true;
    best_err = 1000;
    tol = 0.02;
  }
}

void Twiddle::UpdateError(double cte) {
  if(this->prev_cte == -1)
    prev_cte = 0.0;
  this->cte = cte;
  this->sum_cte = this->sum_cte + cte; 
  this->dt_cte = cte - prev_cte;

  this->Kd = this->p[2];
  this->Ki = this->p[1];
  this->Kp = this->p[0];
}

double Twiddle::TotalError() {
  double angle = -this->Kp*cte - this->Kd*this->dt_cte - this->Ki*this->sum_cte;
  this->prev_cte = this->cte;
  if(angle > 1 || angle < -1 ){
    cout << "WARNING !!!!! \n WARNING !!! \n WARNING !!!\n WError: Angle Out of Range value: " << angle << endl;
    angle = 0.5;
  }
  return angle;
}

double Twiddle::Train(double cte) {
    it += 1;
    if(dp[0]+dp[1]+dp[2] > tol){
      cout << "\n\nIteration " << it << ", best error = " << best_err << endl;
      cout << "INdex: " << i << "<----------- \n"<< endl;
      cout << "P: " << p[0] << ", I: " << p[1] << ", D: " << p[2] << endl;
      cout << "EP: " << dp[0] << ", EI: " << dp[1] << ", ED: " << dp[2] << endl;

      // for(int i=0; i < 3 ; i++){
          p[this->i] += dp[i];
          if(flag_1){
            UpdateError(cte);
            err = TotalError();
          }

          if(err < best_err && flag_1){
            best_err = err;
            dp[i] *= 1.1;

            i+=1;
            i = i % 3;
            return err;
          }else{
            if(flag_1) {
              flag_1 = false;
              return err;
            }

            p[i] -= 2 * dp[i];
            UpdateError(cte);
            err = TotalError();
            if(err < best_err){
              best_err = err;
              dp[i] *= 1.1;
            }else{
              p[i] += dp[i];
              dp[i] *= 0.9;
            }

            i+=1;
            i = i % 3;
            flag_1 = true;
            return err;
          }

      // }
      

    } else {
      cout <<  "No Training!!" << endl;
    }


}
