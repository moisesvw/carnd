#include "PID.h"
#include <random>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  if(!initialized){
    cout << "stst" << endl;
    initialized = true;
    random_device rd;
    default_random_engine gen(rd());
    this->gen = gen;
    normal_distribution<double> steer_dist(-1.3, 10.0);
    this->steer_dist = steer_dist;
  }
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

double PID::next_steer() {
  return this->steer_dist(this->gen);
}
