#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "pid.h"
#include "references.h"

namespace mrs_multirotor_simulator
{

class PositionController {

public:
  class Params {
  public:
    double kp           = 1.0;
    double kd           = 0.5;
    double ki           = 0.01;
    double max_velocity = 4.0;  // m/s
  };

  PositionController();
  PositionController(const MultirotorModel::ModelParams& model_params);

  void setParams(const Params& params);

  reference::Velocity getControlSignal(const MultirotorModel::State& state, const reference::Position& reference, const double& dt);

private:
  MultirotorModel::ModelParams model_params_;
  Params                       params_;

  void initializePIDs(void);

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

}  // namespace mrs_multirotor_simulator

#include "impl/position_controller_impl.hpp"

#endif  // POSITION_CONTROLLER_H
