#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include "pid.h"
#include "references.h"

namespace mrs_multirotor_simulator
{

class VelocityController {

public:
  struct Params
  {
    double kp               = 1.0;
    double kd               = 0.025;
    double ki               = 0.01;
    double max_acceleration = 2.0;  // m/s^2;
  };

  VelocityController();
  VelocityController(const ModelParams& model_params);

  void setParams(const Params& params);

  reference::Acceleration getControlSignal(const MultirotorModel::State& state, const reference::Velocity& reference, const double& dt);

private:
  ModelParams model_params_;
  Params      params_;

  void initializePIDs(void);

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

}  // namespace mrs_multirotor_simulator

#include "impl/velocity_controller_impl.hpp"

#endif  // VELOCITY_CONTROLLER_H
