#ifndef RATE_CONTROLLER_H
#define RATE_CONTROLLER_H

#include "pid.h"
#include "references.h"

namespace mrs_multirotor_simulator
{

class RateController {

public:
  class Params {
  public:
    double kp = 4.0;
    double kd = 0.04;
    double ki = 0.0;
  };

  RateController();
  RateController(const MultirotorModel::ModelParams& model_params);

  void setParams(const Params& params);

  reference::ControlGroup getControlSignal(const MultirotorModel::State& state, const reference::AttitudeRate& reference, const double& dt);

private:
  MultirotorModel::ModelParams model_params_;
  Params                       params_;

  void initializePIDs(void);

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

}  // namespace mrs_multirotor_simulator

#include "impl/rate_controller_impl.hpp"

#endif  // RATE_CONTROLLER_H
