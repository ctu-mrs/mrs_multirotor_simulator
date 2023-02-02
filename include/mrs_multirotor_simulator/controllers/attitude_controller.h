#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <mrs_multirotor_simulator/multirotor_model.h>
#include <mrs_multirotor_simulator/controllers/pid.h>
#include <mrs_multirotor_simulator/controllers/references.h>

namespace mrs_multirotor_simulator
{

class AttitudeController {

public:
  class Params {
  public:
    double kp                  = 4.0;
    double kd                  = 0.05;
    double ki                  = 0.01;
    double max_rate_roll_pitch = 10.0;  // rad/s
    double max_rate_yaw        = 1.0;   // rad/s
  };

  AttitudeController();
  AttitudeController(const ModelParams& model_params);

  void setParams(const Params& params);

  reference::AttitudeRate getControlSignal(const MultirotorModel::State& state, const reference::Attitude& reference, const double& dt);

private:
  Params      params_;
  ModelParams model_params_;

  void initializePIDS(void);

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

}  // namespace mrs_multirotor_simulator

#endif  // ATTITUDE_CONTROLLER_H
