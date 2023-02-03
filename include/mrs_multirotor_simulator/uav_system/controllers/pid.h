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

  double saturation = -1;

public:
  PIDController();

  void setParams(const double kp, const double kd, const double ki, const double saturation = -1);

  void reset(void);

  double update(double error, double dt);
};

}  // namespace mrs_multirotor_simulator

#include "impl/pid_impl.hpp"

#endif  // PID_H
