#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <mrs_multirotor_simulator/multirotor_model.h>
#include <mrs_multirotor_simulator/controllers/pid.h>
#include <mrs_multirotor_simulator/controllers/references.h>

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
    double max_rate_roll_pitch;
    double max_rate_yaw;
  };

  AttitudeController();

  void setParams(const Params& params);

  reference::AttitudeRate getControlSignal(const MultirotorModel::State& state, const reference::Attitude& reference, const double& dt);

private:
  Params params_;

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

}  // namespace mrs_multirotor_simulator

#endif  // ATTITUDE_CONTROLLER_H
