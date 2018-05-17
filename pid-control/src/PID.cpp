#include "PID.h"
#include <random>
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  if(!initialized){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    initialized = true;
    random_device rd;
    default_random_engine gen(rd());
    this->gen = gen;
    normal_distribution<double> steer_dist(0.0, 0.6);
    this->steer_dist = steer_dist;
  }
}

void PID::UpdateError(double cte) {
  if(this->prev_cte == -1)
    prev_cte = 0.0;
  this->cte = cte;
  this->sum_cte = this->sum_cte + cte; 
  this->dt_cte = cte - prev_cte;
}

double PID::TotalError() {
  double angle = -this->Kp*cte - this->Kd*this->dt_cte - this->Ki*this->sum_cte;
  this->prev_cte = this->cte;
  if(angle > 1 || angle < -1 ){
    cout << "WARNING !!!!! \n WARNING !!! \n WARNING !!!\n WError: Angle Out of Range value: " << angle << endl;
    angle = steer_dist(gen);
  }
  return angle;
}
