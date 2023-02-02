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

#endif  // VELOCITY_CONTROLLER_H
