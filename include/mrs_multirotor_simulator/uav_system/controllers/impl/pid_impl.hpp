#ifndef PID_IMPL_H
#define PID_IMPL_H

namespace mrs_multirotor_simulator
{

PIDController::PIDController() {

  this->reset();
}

void PIDController::setParams(const double kp, const double kd, const double ki, const double saturation) {

  this->_kp_       = kp;
  this->_kd_       = kd;
  this->_ki_       = ki;
  this->saturation = saturation;
}

void PIDController::reset(void) {

  this->last_error_ = 0;
  this->integral_   = 0;
}

double PIDController::update(double error, double dt) {

  // calculate the control error difference
  double difference = (error - last_error_) / dt;
  last_error_       = error;

  double p_component = _kp_ * error;       // proportional feedback
  double d_component = _kd_ * difference;  // derivative feedback
  double i_component = _ki_ * integral_;   // derivative feedback

  double sum = p_component + d_component + i_component;

  if (saturation > 0) {

    if (sum > saturation) {
      sum = saturation;
    } else if (sum < -saturation) {
      sum = -saturation;
    } else {
      integral_ += error * dt;
    }

  } else {

    // add to the integral
    integral_ += error * dt;
  }

  // return the summ of the components
  return sum;
}

}  // namespace mrs_multirotor_simulator

#endif  // PID_IMPL_H
