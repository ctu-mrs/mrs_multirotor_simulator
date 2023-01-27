#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <quadrotor_model.h>
#include <controllers/pid.h>
#include <controllers/references.h>

namespace mrs_multirotor_simulator
{

class AttitudeController {

public:
  struct Params
  {
    double kp;
    double kd;
    double ki;
    double mass;
  };

  AttitudeController();

  void setParams(const Params& params);

  reference::AttitudeRate getControlSignal(const QuadrotorModel::State& state, const reference::Attitude& reference, const double& dt);

private:
  Params params_;

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

}  // namespace mrs_multirotor_simulator

#endif  // ATTITUDE_CONTROLLER_H
