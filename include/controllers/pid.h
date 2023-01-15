#ifndef PID_H
#define PID_H

#include <math.h>

namespace mrs_multirotor_simulator
{

class PIDController {

private:
  // | ----------------------- parameters ----------------------- |

  // gains
  double _kp_ = 0;  // proportional gain
  double _kd_ = 0;  // derivative gain
  double _ki_ = 0;  // integral gain

  // we remember the last control error, to calculate the difference
  double last_error_ = 0;
  double integral_   = 0;

public:
  PIDController() {

    this->reset();
  }

  void setParams(const double kp, const double kd, const double ki) {

    this->_kp_ = kp;
    this->_kd_ = kd;
    this->_ki_ = ki;
  }

  void reset(void) {

    this->last_error_ = 0;
    this->integral_   = 0;
  }

  double update(double error, double dt) {

    // calculate the control error difference
    double difference_ = (error - last_error_) / dt;
    last_error_        = error;

    // add to the integral
    integral_ += error * dt;

    double p_component = _kp_ * error;        // proportional feedback
    double d_component = _kd_ * difference_;  // derivative feedback
    double i_component = _ki_ * integral_;    // derivative feedback

    // return the summ of the components
    return p_component + d_component + i_component;
  }
};

}  // namespace mrs_multirotor_simulator

#endif  // PID_H
