#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <quadrotor_model.h>
#include <controllers/pid.h>
#include <controllers/references.h>

namespace mrs_multirotor_simulator
{

class PositionController {

public:
  struct Params
  {
    double kp;
    double kd;
    double ki;
  };

  PositionController();

  void setParams(const Params& params);

  reference::Velocity getControlSignal(const QuadrotorModel::State& state, const reference::Position& reference, const double& dt);

private:
  Params params_;

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

}  // namespace mrs_multirotor_simulator

#endif  // POSITION_CONTROLLER_H
