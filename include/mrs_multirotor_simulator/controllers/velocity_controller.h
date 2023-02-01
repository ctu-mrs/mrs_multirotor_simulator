#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include <mrs_multirotor_simulator/multirotor_model.h>
#include <mrs_multirotor_simulator/controllers/pid.h>
#include <mrs_multirotor_simulator/controllers/references.h>

namespace mrs_multirotor_simulator
{

class VelocityController {

public:
  struct Params
  {
    double kp;
    double kd;
    double ki;
    double max_acceleration;
  };

  VelocityController();

  void setParams(const Params& params);

  reference::Acceleration getControlSignal(const MultirotorModel::State& state, const reference::Velocity& reference, const double& dt);

private:
  Params params_;

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

}  // namespace mrs_multirotor_simulator

#endif  // VELOCITY_CONTROLLER_H
