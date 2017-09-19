#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // initialize error parameters
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  // initialize gain parameters
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

}

void PID::UpdateError(double cte) {
  // assuming unit time step per error
  // diffrential error
  d_error = cte - p_error;
  // proportional error
  p_error = cte;
  // integral error
  i_error += cte;
}

double PID::TotalError() {
  // compute correction
  double steer;
  steer = -Kp * p_error - Kd * d_error - Ki * i_error;

  return steer;
}

