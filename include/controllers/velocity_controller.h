#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include <quadrotor_model.h>
#include <controllers/pid.h>
#include <controllers/references.h>

namespace mrs_multirotor_simulator
{

class VelocityController {

public:
  struct Params
  {
    double kp;
    double kd;
    double ki;
  };

  VelocityController();

  void setParams(const Params& params);

  reference::Acceleration getControlSignal(const QuadrotorModel::State& state, const reference::Velocity& reference, const double& dt);

private:
  Params params_;

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

}  // namespace mrs_multirotor_simulator

#endif  // VELOCITY_CONTROLLER_H
